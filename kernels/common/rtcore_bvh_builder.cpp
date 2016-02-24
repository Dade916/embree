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

RTCORE_API void *rtcBVHBuilderAllocator(void *allocator, const size_t size) {
	FastAllocator::ThreadLocal *alloc = (FastAllocator::ThreadLocal *)allocator;

	return alloc->malloc(size);
}

RTCORE_API void *rtcBVHBuilderBinnedSAH(const RTCPrimRef *prims, const size_t primRefsSize,
		rtcBVHBuilderNodeAllocFunc nodeAllocFunc,
		rtcBVHBuilderLeafAllocFunc leafAllocFunc,
		rtcBVHBuilderNodeChildrenPtrFunc nodeChildrenPtrFunc) {
	/* fast allocator that supports thread local operation */
	FastAllocator allocator(nullptr);

	isa::PrimInfo primsInfo(empty);
	for (size_t i = 0; i < primRefsSize; i++) {
		const BBox3fa bbox(Vec3fa::load(&prims[i].lower_x), Vec3fa::load(&prims[i].upper_x));
		primsInfo.add(bbox);
	}

	void *root;
	isa::BVHBuilderBinnedSAH::build<void *>(
			root,
			/* thread local allocator for fast allocations */
			[&] () -> FastAllocator::ThreadLocal * {
				return allocator.threadLocal();
			},

			/* lambda function that creates BVH nodes */
			[&](const isa::BVHBuilderBinnedSAH::BuildRecord &current, isa::BVHBuilderBinnedSAH::BuildRecord *children, const size_t N, FastAllocator::ThreadLocal *alloc) -> int {
				assert(N <= 2);
				
				void *node = (*nodeAllocFunc)(alloc);
				for (size_t i = 0; i < N; i++)
					children[i].parent = (size_t *)nodeChildrenPtrFunc(node, i);
				*current.parent = (size_t)node;

				return 0;
			},

			/* lambda function that creates BVH leaves */
			[&](const isa::BVHBuilderBinnedSAH::BuildRecord &current, FastAllocator::ThreadLocal *alloc) -> int {
				assert(current.prims.size() == 1);

				void *node = (*leafAllocFunc)(alloc, &prims[current.prims.begin()]);
				*current.parent = (size_t)node;

				return 0;
			},

			/* progress monitor function */
			[&] (size_t dn) {
				// throw an exception here to cancel the build operation
			},

			(PrimRef *)prims, primsInfo, 2, 1024, 1, 1, 1, 1.f, 1.f);

	return NULL;
}

}