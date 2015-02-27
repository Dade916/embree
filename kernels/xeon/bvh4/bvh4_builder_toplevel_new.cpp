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

#include "bvh4_builder_toplevel_new.h"

namespace embree
{
  namespace isa
  {
#define MIN_OPEN_SIZE 2000

    BVH4BuilderTopLevelNew::BVH4BuilderTopLevelNew (BVH4* bvh, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel) 
      : bvh(bvh), objects(bvh->objects), scene(scene), createTriangleMeshAccel(createTriangleMeshAccel) {}
    
    BVH4BuilderTopLevelNew::~BVH4BuilderTopLevelNew ()
    {
      for (size_t i=0; i<builders.size(); i++) 
	delete builders[i];
    }

    void BVH4BuilderTopLevelNew::build(size_t threadIndex, size_t threadCount) 
    {
      /* delete some objects */
      size_t N = scene->size();
      parallel_for(N, objects.size(), [&] (const range<size_t>& r) {
        for (size_t i=r.begin(); i<r.end(); i++) {
          delete builders[i]; builders[i] = NULL;
          delete objects[i]; objects[i] = NULL;
        }
      });

      /* skip build for empty scene */
      const size_t numPrimitives = scene->getNumPrimitives<TriangleMesh,1>();
      if (numPrimitives == 0) {
        prims.resize(0);
        bvh->set(BVH4::emptyNode,empty,0);
        return;
      }
      
      double t0 = 0.0, dt = 0.0;
      if (g_verbose >= 1) {
	std::cout << "building BVH4<" << bvh->primTy.name << "> with " << TOSTRING(isa) << "::TwoLevel SAH builder ... " << std::flush;
        t0 = getSeconds();
      }

      /* resize object array if scene got larger */
      if (objects.size() < N) {
        objects.resize(N);
        builders.resize(N);
        refs.resize(N);
      }
      nextRef = 0;
      
      /* create of acceleration structures */
      parallel_for(size_t(0), N, [&] (const range<size_t>& r) 
      {
        for (size_t objectID=r.begin(); objectID<r.end(); objectID++)
        {
          TriangleMesh* mesh = scene->getTriangleMeshSafe(objectID);
          
          /* verify meshes got deleted properly */
          if (mesh == NULL || mesh->numTimeSteps != 1) {
            assert(objectID < objects.size () && objects[objectID] == NULL);
            assert(objectID < builders.size() && builders[objectID] == NULL);
            continue;
          }
          
          /* delete BVH and builder for meshes that are scheduled for deletion */
          if (mesh->state == Geometry::ERASING) {
            delete builders[objectID]; builders[objectID] = NULL;
            delete objects [objectID]; objects [objectID] = NULL;
            continue;
          }
          
          /* create BVH and builder for new meshes */
          if (objects[objectID] == NULL)
            createTriangleMeshAccel(mesh,objects[objectID],builders[objectID]);
        }
      });
      
      /* parallel build of acceleration structures */
      parallel_for(size_t(0), N, [&] (const range<size_t>& r) 
      {
        for (size_t objectID=r.begin(); objectID<r.end(); objectID++)
        {
          /* ignore if no triangle mesh or not enabled */
          TriangleMesh* mesh = scene->getTriangleMeshSafe(objectID);
          if (mesh == NULL || !mesh->isEnabled() || mesh->numTimeSteps != 1) 
            continue;
        
          BVH4*    object  = objects [objectID]; assert(object);
          Builder* builder = builders[objectID]; assert(builder);
          
          /* build object if it got modified */
          if (mesh->isModified()) {
            builder->build(0,0);
            mesh->state = Geometry::ENABLED;
          }
          
          /* create build primitive */
          if (!object->bounds.empty())
            refs[nextRef++] = BuildRef(object->bounds,object->root);
        }
      });
      
      refs.resize(nextRef);

      /* open all large nodes */
      open_sequential();
      prims.resize(refs.size());

      /* compute PrimRefs */
      const PrimInfo pinfo = parallel_reduce(size_t(0), refs.size(), size_t(1024), PrimInfo(empty), [&] (const range<size_t>& r)
      {
        PrimInfo pinfo(empty);
        for (size_t i=r.begin(); i<r.end(); i++) {
          pinfo.add(refs[i].bounds());
          prims[i] = PrimRef(refs[i].bounds(),(size_t)refs[i].node);
        }
        return pinfo;
      }, [] (const PrimInfo& a, const PrimInfo& b) { return PrimInfo::merge(a,b); });

      /* skip if all objects where empty */
      if (pinfo.size() == 0)
        bvh->set(BVH4::emptyNode,empty,0);

      /* otherwise build toplevel hierarchy */
      else
      {
        BVH4::NodeRef root = bvh_builder_binned_sah_internal<BVH4::NodeRef>
          ([&] { return bvh->alloc2.threadLocal2(); },
           [&] (const isa::BuildRecord<BVH4::NodeRef>& current, BuildRecord<BVH4::NodeRef>** children, const size_t N, FastAllocator::ThreadLocal2* alloc) 
           {
             BVH4::Node* node = (BVH4::Node*) alloc->alloc0.malloc(sizeof(BVH4::Node)); node->clear();
             for (size_t i=0; i<N; i++) {
               node->set(i,children[i]->geomBounds);
               children[i]->parent = &node->child(i);
             }
             *current.parent = bvh->encodeNode(node);
             return 0;
           },
           [&] (const BuildRecord<BVH4::NodeRef>& current, FastAllocator::ThreadLocal2* alloc) // FIXME: why are prims passed here but not for createNode
           {
             assert(current.prims.size() == 1);
             *current.parent = (BVH4::NodeRef) prims[current.prims.begin()].ID();
             return 1;
           },
           prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,1,1,1);
        
        bvh->set(root,pinfo.geomBounds,numPrimitives);
      }

      if (g_verbose >= 1) {
        dt = getSeconds()-t0;
        std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mprim/s)" << std::endl;
      }
      if (g_verbose >= 2)
        bvh->printStatistics();
    }

    void BVH4BuilderTopLevelNew::open_sequential()
    {
      if (refs.size() == 0)
	return;

      size_t N = max(2*refs.size(),size_t(MIN_OPEN_SIZE));
      refs.reserve(N);
      
      std::make_heap(refs.begin(),refs.end());
      while (refs.size()+3 <= N)
      {
        std::pop_heap (refs.begin(),refs.end()); 
        BVH4::NodeRef ref = refs.back().node;
        if (ref.isLeaf()) break;
        refs.pop_back();    
        
        BVH4::Node* node = ref.node();
        for (size_t i=0; i<4; i++) {
          if (node->child(i) == BVH4::emptyNode) continue;
          refs.push_back(BuildRef(node->bounds(i),node->child(i)));
          std::push_heap (refs.begin(),refs.end()); 
        }
      }
    }
    
    Builder* BVH4BuilderTopLevelBinnedSAH (BVH4* bvh, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel) {
      return new BVH4BuilderTopLevelNew(bvh,scene,createTriangleMeshAccel);
    }
  }
}