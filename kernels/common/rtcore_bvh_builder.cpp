// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#ifdef _WIN32
#  define RTCORE_API extern "C" __declspec(dllexport)
#else
#  define RTCORE_API extern "C" __attribute__ ((visibility ("default")))
#endif

#include "../../include/embree2/rtcore.h"
#include "../../include/embree2/rtcore_bvh_builder.h"

#include "alloc.h"
#include "../xeon/builders/bvh_builder_sah.h"
#include "../xeon/builders/bvh_builder_morton.h"

namespace embree {

//------------------------------------------------------------------------------
// Memory related functions
//------------------------------------------------------------------------------

RTCORE_API RTCAllocator rtcNewAllocator() {
	return (RTCAllocator) new FastAllocator(nullptr);
}

RTCORE_API void rtcDeleteAllocator(RTCAllocator allocator) {
	FastAllocator *fastAllocator = (FastAllocator *)allocator;
	delete fastAllocator;
}

RTCORE_API void rtcResetAllocator(RTCAllocator allocator) {
	FastAllocator *fastAllocator = (FastAllocator *)allocator;
	fastAllocator->reset();
}

RTCORE_API RTCThreadLocalAllocator rtcNewThreadAllocator(RTCAllocator allocator) {
	FastAllocator *fastAllocator = (FastAllocator *)allocator;

	return (RTCThreadLocalAllocator) fastAllocator->threadLocal();
}

RTCORE_API void *rtcThreadAlloc(RTCThreadLocalAllocator allocator, const size_t size) {
	FastAllocator::ThreadLocal *threadAllocator = (FastAllocator::ThreadLocal *)allocator;

	return threadAllocator->malloc(size);
}

//------------------------------------------------------------------------------
// BVH builder related functions
//------------------------------------------------------------------------------

RTCORE_API void rtcDefaultBVHBuilderConfig(RTCBVHBuilderConfig *config) {
	config->branchingFactor = 2;
	config->maxDepth = 1024;
	config->blockSize = 1;
	config->minLeafSize = 1;
	config->maxLeafSize = 1;
	config->travCost = 1.f;
	config->intCost = 1.f;
}

RTCORE_API void *rtcBVHBuilderBinnedSAH(const RTCBVHBuilderConfig *config, 
		const RTCPrimRef *prims, const size_t primRefsSize, void *userGlobalData,
		rtcBVHBuilderCreateLocalThreadDataFunc createLocalThreadDataFunc,
		rtcBVHBuilderCreateNodeFunc createNodeFunc,
		rtcBVHBuilderCreateLeafFunc createLeafFunc,
		rtcBVHBuilderGetNodeChildrenPtrFunc getNodeChildrenPtrFunc,
		rtcBVHBuilderGetNodeChildrenBBoxFunc getNodeChildrenBBoxFunc) {
	isa::PrimInfo primsInfo(empty);
	for (size_t i = 0; i < primRefsSize; i++) {
		const BBox3fa bbox(Vec3fa::load(&prims[i].lower_x), Vec3fa::load(&prims[i].upper_x));
		primsInfo.add(bbox);
	}

	void *root;
	isa::BVHBuilderBinnedSAH::build<void *>(
			root,
			/* thread local allocator for fast allocations */
			[&] () -> void * {
				return createLocalThreadDataFunc(userGlobalData);
			},

			/* lambda function that creates BVH nodes */
			[&](const isa::BVHBuilderBinnedSAH::BuildRecord &current, isa::BVHBuilderBinnedSAH::BuildRecord *children, const size_t N, void *userLocalThreadData) -> int {
				void *node = (*createNodeFunc)(userLocalThreadData);
				for (size_t i = 0; i < N; i++) {
					getNodeChildrenBBoxFunc(node, i, &children[i].pinfo.geomBounds.lower.x, &children[i].pinfo.geomBounds.upper.x);
					children[i].parent = (size_t *)getNodeChildrenPtrFunc(node, i);
				}
				*current.parent = (size_t)node;

				return 0;
			},

			/* lambda function that creates BVH leaves */
			[&](const isa::BVHBuilderBinnedSAH::BuildRecord &current, void *userLocalThreadData) -> int {
				assert(current.prims.size() == 1);

				const RTCPrimRef *prim = &prims[current.prims.begin()];
				void *leaf = (*createLeafFunc)(userLocalThreadData, prim->geomID, prim->primID, &prim->lower_x, &prim->upper_x);
				*current.parent = (size_t)leaf;

				return 0;
			},

			/* progress monitor function */
			[&] (size_t dn) {
				// throw an exception here to cancel the build operation
			},

			(PrimRef *)prims, primsInfo,
			config->branchingFactor,
			config->maxDepth,
			config->blockSize,
			config->minLeafSize,
			config->maxLeafSize,
			config->travCost,
			config->intCost);

	return root;
}

RTCORE_API void *rtcBVHBuilderMorton(const RTCBVHBuilderConfig *config, 
		const RTCPrimRef *prims, const size_t primRefsSize, void *userGlobalData,
		rtcBVHBuilderCreateLocalThreadDataFunc createLocalThreadDataFunc,
		rtcBVHBuilderCreateNodeFunc createNodeFunc,
		rtcBVHBuilderCreateLeafFunc createLeafFunc,
		rtcBVHBuilderGetNodeChildrenPtrFunc getNodeChildrenPtrFunc,
		rtcBVHBuilderGetNodeChildrenBBoxFunc getNodeChildrenBBoxFunc) {
	/* array for morton builder */
	avector<isa::MortonID32Bit> morton_src(primRefsSize);
	avector<isa::MortonID32Bit> morton_tmp(primRefsSize);
	for (size_t i = 0; i < primRefsSize; i++)
		morton_src[i].index = i;

	std::pair<void *, BBox3fa> node_bounds = isa::bvh_builder_morton<void *>(

			/* thread local allocator for fast allocations */
			[&] () -> void * {
				return createLocalThreadDataFunc(userGlobalData);
			},

			BBox3fa(empty),

			/* lambda function that allocates BVH nodes */
			[&] (isa::MortonBuildRecord<void *>& current, isa::MortonBuildRecord<void *> *children, size_t N, void *userLocalThreadData) -> void * {
				void *node = (*createNodeFunc)(userLocalThreadData);
				for (size_t i = 0; i < N; i++)
					children[i].parent = (void **)getNodeChildrenPtrFunc(node, i);
				*current.parent = node;

				return node;
		},

		/* lambda function that sets bounds */
		[&] (void *node, const BBox3fa *bounds, size_t N) -> BBox3fa {
			BBox3fa res = empty;
			for (size_t i = 0; i < N; i++) {
				const BBox3fa b = bounds[i];
				res.extend(b);
				
				getNodeChildrenBBoxFunc(node, i, &b.lower.x, &b.upper.x);
			}

			return res;
		},

		/* lambda function that creates BVH leaves */
		[&](isa::MortonBuildRecord<void *>& current, void *userLocalThreadData, BBox3fa &box_o) -> void * {
			const size_t id = morton_src[current.begin].index;
			const BBox3fa bounds(Vec3fa::load(&prims[id].lower_x), Vec3fa::load(&prims[id].upper_x));

			void *leaf = (*createLeafFunc)(userLocalThreadData, int(id >> 32), int(id & 0xffffffff), &bounds.lower.x, &bounds.upper.x);

			*current.parent = leaf;
			box_o = bounds;
			return leaf;
		},

		/* lambda that calculates the bounds for some primitive */
		[&] (const isa::MortonID32Bit &morton) -> BBox3fa {
			const BBox3fa bounds(Vec3fa::load(&prims[morton.index].lower_x), Vec3fa::load(&prims[morton.index].upper_x));

			return bounds;
		},

		/* progress monitor function */
		[&] (size_t dn) {
			// throw an exception here to cancel the build operation
		},

		morton_src.data(), morton_tmp.data(), primRefsSize,
		config->branchingFactor,
		config->maxDepth,
		config->minLeafSize,
		config->maxLeafSize);

	return node_bounds.first;
}

}