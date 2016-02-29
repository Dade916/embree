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

#include <functional>

#include "../common/tutorial/tutorial_device.h"
#include "../../kernels/common/alloc.h"
#include "../../include/embree2/rtcore_bvh_builder.h"

RTCDevice g_device = nullptr;
RTCScene g_scene  = nullptr;

/* render function to use */
renderPixelFunc renderPixel;

/* error reporting function */
void error_handler(const RTCError code, const char* str = NULL)
{
  if (code == RTC_NO_ERROR) 
    return;

  printf("Embree: ");
  switch (code) {
  case RTC_UNKNOWN_ERROR    : printf("RTC_UNKNOWN_ERROR"); break;
  case RTC_INVALID_ARGUMENT : printf("RTC_INVALID_ARGUMENT"); break;
  case RTC_INVALID_OPERATION: printf("RTC_INVALID_OPERATION"); break;
  case RTC_OUT_OF_MEMORY    : printf("RTC_OUT_OF_MEMORY"); break;
  case RTC_UNSUPPORTED_CPU  : printf("RTC_UNSUPPORTED_CPU"); break;
  case RTC_CANCELLED        : printf("RTC_CANCELLED"); break;
  default                   : printf("invalid error code"); break;
  }
  if (str) { 
    printf(" ("); 
    while (*str) putchar(*str++); 
    printf(")\n"); 
  }
  exit(1);
}

/* These function called by the builder to signal progress and to
 * report memory consumption. */
namespace embree
{
  void memoryMonitor(ssize_t bytes, bool post)
  {
    // throw an exception here when nprims>0 to cancel the build operation
  }
}

struct Node
{
  virtual float sah() = 0;
};

struct InnerNode : public Node
{
  BBox3fa bounds[2];
  Node* children[2];

  InnerNode() {
    bounds[0] = bounds[1] = empty;
    children[0] = children[1] = nullptr;
  }
  
  float sah() {
    return 1.f + (area(bounds[0])*children[0]->sah() + area(bounds[1])*children[1]->sah())/area(merge(bounds[0],bounds[1]));
  }
};

struct LeafNode : public Node
{
  size_t id;
  BBox3fa bounds;

  LeafNode (size_t id, const BBox3fa& bounds)
    : id(id), bounds(bounds) {}

  float sah() {
    return 1.f;
  }
};

//------------------------------------------------------------------------------

static void *CreateAllocFunc(void *userData) {
	RTCAllocator fastAllocator = *((RTCAllocator *)userData);

	return rtcNewThreadAllocator(fastAllocator);
}

static void *CreateNodeFunc(void *localAllocator) {
	RTCThreadLocalAllocator fastLocalAllocator = (RTCThreadLocalAllocator)localAllocator;
	return new (rtcThreadAlloc(fastLocalAllocator, sizeof(InnerNode))) InnerNode();
}

static void *CreateLeafFunc(void *localAllocator, const RTCPrimRef *prim) {
	RTCThreadLocalAllocator fastLocalAllocator = (RTCThreadLocalAllocator)localAllocator;
	const BBox3fa bbox(Vec3fa::load(&prim->lower_x), Vec3fa::load(&prim->upper_x));

	return new (rtcThreadAlloc(fastLocalAllocator, sizeof(LeafNode))) LeafNode(size_t(prim->geomID) + (size_t(prim->primID) << 32), bbox);
}

static void *NodeChildrenPtrFunc(void *n, const size_t i) {
	InnerNode *node = (InnerNode *)n;
	return &node->children[i];
}

static void NodeChildrenSetBBoxFunc(void *n, const size_t i, const float lower[3], const float upper[3]) {
	InnerNode *node = (InnerNode *)n;

	node->bounds[i].lower.x = lower[0];
	node->bounds[i].lower.y = lower[1];
	node->bounds[i].lower.z = lower[2];

	node->bounds[i].upper.x = upper[0];
	node->bounds[i].upper.y = upper[1];
	node->bounds[i].upper.z = upper[2];
}

void build_sah(avector<RTCPrimRef>& prims) {
	RTCAllocator fastAllocator = rtcNewAllocator();

	for (size_t i = 0; i < 2; i++) {
		std::cout << "iteration " << i << ": building BVH over " << prims.size() << " primitives, " << std::flush;
		double t0 = getSeconds();

		rtcResetAllocator(fastAllocator);

		RTCBVHBuilderConfig config;
		rtcDefaultBVHBuilderConfig(&config);

		Node *root = (Node *) rtcBVHBuilderBinnedSAH(&config,
				&prims[0], prims.size(),
				&fastAllocator,
				&CreateAllocFunc, &CreateNodeFunc, &CreateLeafFunc,
				&NodeChildrenPtrFunc, &NodeChildrenSetBBoxFunc);

		double t1 = getSeconds();
		std::cout << 1000.0f * (t1 - t0) << "ms, " << 1E-6 * double(prims.size()) / (t1 - t0) << " Mprims/s, sah = " << root->sah() << " [DONE]" << std::endl;
	}

	rtcDeleteAllocator(fastAllocator);
}

//------------------------------------------------------------------------------

static void *CreateAllocSystemAllocatorFunc(void *userData) {
	return NULL;
}

static void *CreateNodeSystemAllocatorFunc(void *localAllocator) {
	return new InnerNode();
}

static void *CreateLeafSystemAllocatorFunc(void *localAllocator, const RTCPrimRef *prim) {
	const BBox3fa bbox(Vec3fa::load(&prim->lower_x), Vec3fa::load(&prim->upper_x));

	return new LeafNode(size_t(prim->geomID) + (size_t(prim->primID) << 32), bbox);
}

static void FreeTree(Node *node) {
	InnerNode *innerNode = dynamic_cast<InnerNode *>(node);
	if (innerNode) {
		FreeTree(innerNode->children[0]);
		FreeTree(innerNode->children[1]);
	}

	delete node;
}

void build_sah_system_memory(avector<RTCPrimRef>& prims) {
	for (size_t i = 0; i < 2; i++) {
		std::cout << "iteration " << i << ": building BVH over " << prims.size() << " primitives, " << std::flush;
		double t0 = getSeconds();

		RTCBVHBuilderConfig config;
		rtcDefaultBVHBuilderConfig(&config);

		Node *root = (Node *) rtcBVHBuilderBinnedSAH(&config,
				&prims[0], prims.size(),
				NULL,
				&CreateAllocSystemAllocatorFunc, &CreateNodeSystemAllocatorFunc, &CreateLeafSystemAllocatorFunc,
				&NodeChildrenPtrFunc, &NodeChildrenSetBBoxFunc);

		double t1 = getSeconds();
		std::cout << 1000.0f * (t1 - t0) << "ms, " << 1E-6 * double(prims.size()) / (t1 - t0) << " Mprims/s, sah = " << root->sah() << " [DONE]" << std::endl;

		// Free the allocated memory
		FreeTree(root);
	}
}

//------------------------------------------------------------------------------

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler);
  
  /* set start render mode */
  renderPixel = renderPixelStandard;

  /* create random bounding boxes */
  const size_t N = 2300000;
  avector<RTCPrimRef> prims; 
  for (size_t i=0; i<N; i++) {
    const Vec3fa p = 1000.0f*Vec3fa(drand48(),drand48(),drand48());
    const BBox3fa b = BBox3fa(p,p+Vec3fa(1.0f));

    RTCPrimRef prim;
	prim.lower_x = b.lower.x;
	prim.lower_y = b.lower.y;
	prim.lower_z = b.lower.z;
	prim.geomID = 0;
	prim.upper_x = b.upper.x;
	prim.upper_y = b.upper.y;
	prim.upper_z = b.upper.z;
	prim.primID = i;

    prims.push_back(prim);
  }

  std::cout << "Build SAH with system memory allocator" << std::endl;
  build_sah_system_memory(prims);
  std::cout << "Build SAH with RTC allocator" << std::endl;
  build_sah(prims);
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  return Vec3fa(zero);
}

/* task that renders a single screen tile */
void renderTile(int taskIndex, int* pixels,
                     const int width,
                     const int height, 
                     const float time,
                     const Vec3fa& vx, 
                     const Vec3fa& vy, 
                     const Vec3fa& vz, 
                     const Vec3fa& p,
                     const int numTilesX, 
                     const int numTilesY)
{
  const int tileY = taskIndex / numTilesX;
  const int tileX = taskIndex - tileY * numTilesX;
  const int x0 = tileX * TILE_SIZE_X;
  const int x1 = min(x0+TILE_SIZE_X,width);
  const int y0 = tileY * TILE_SIZE_Y;
  const int y1 = min(y0+TILE_SIZE_Y,height);

  for (int y = y0; y<y1; y++) for (int x = x0; x<x1; x++)
  {
    /* calculate pixel color */
    Vec3fa color = renderPixel(x,y,vx,vy,vz,p);
    
    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                    const int width,
                    const int height,
                    const float time,
                    const Vec3fa& vx, 
                    const Vec3fa& vy, 
                    const Vec3fa& vz, 
                    const Vec3fa& p)
{
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch_renderTile(numTilesX*numTilesY,pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); 
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup () {
  rtcDeleteDevice(g_device);
}

