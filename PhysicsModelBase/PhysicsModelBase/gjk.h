//
// gjk algorithm
// by stan melax 2007   open source
//
// Just a basic gjk implementation using some wisdom described in Gino's papers on the algorithm. 
// Essecntially take two convex shapes in the form of a supportmap function/lambda and returns separating plane.
// Implementation is inline for convenience
// 
// 2014 update:  just a bit of code and style cleanup in case anyone is interested in trying/testing/using this
//               generalized the interface so function now takes two functions (or lambdas) 
//               that return the supportmap for a given direction.
//

#pragma once
#ifndef GJK_H
#define GJK_H


#include <assert.h>
#include <functional>

#include "vecmatquat.h"
#include "geometric.h"           // a collection of useful utilities such as intersection and projection


namespace gjk_implementation
{
	class Contact
	{
	public:
		int				type; //  v[t] on c[0] for t<type on c[1] for t>=type so 2 is edge-edge collision
		float3			v[4];
		float3			normal; // worldspace  points from C[1] to C[0]
		float			dist;  // plane
		float3			impact;
		float			separation;
		float3 p0w, p1w;
		float time;
		Contact() :normal(0, 0, 0), impact(0, 0, 0) { type = -1; separation = FLT_MAX; }
		operator bool() { return separation <= 0; }
	};

	struct MKPoint
	{
		float3 a;
		float3 b;
		float3 p;
		float t;
		MKPoint(const float3  &a, const float3  &b, const float3& p) :a(a), b(b), p(p), t(0){}
		MKPoint(){}
	};

	struct MinkSimplex
	{
		float3 v;
		MKPoint W[4];
		int count;
		float3 pa;
		float3 pb;
	};

	inline int operator==(const MKPoint &a,const MKPoint &b)
	{
		return (a.a==b.a && a.b==b.b);
	}

	inline MKPoint PointOnMinkowski(std::function<float3(const float3&)> A, std::function<float3(const float3&)>B,  const float3 &n)
	{
		auto a = A( n);
		auto b = B(-n);
		return MKPoint(a,b,a-b);
	}
	inline MKPoint PointOnMinkowski(std::function<float3(const float3&)> A, std::function<float3(const float3&)>B, const float3& ray, const float3 &n)
	{
		auto p = PointOnMinkowski(A, B, n);
		p.p += ((dot(n, ray) > 0.0f) ? ray : float3(0, 0, 0));
		return p;
	}


	inline int NextMinkSimplex0(MinkSimplex &dst, const MinkSimplex &src, const MKPoint &w)
	{
		assert(src.count==0);
		dst.W[0] = w;
		dst.W[0].t = 1.0f;
		dst.v = w.p;
		// ?? dst.pa = w.a;
		dst.count=1;
		return 1;
	}

	inline int NextMinkSimplex1(MinkSimplex &dst, const MinkSimplex &src, const MKPoint &w)
	{
		assert(src.count==1);
		float t = LineProjectTime(w.p ,src.W[0].p,  float3(0,0,0));
		if(t<0.0f)  // w[0] is useless
		{
			dst.W[0] = w;
			dst.W[0].t=1.0f;
			dst.v = w.p;
			dst.count=1;
			return 1;
		}
		dst.W[0] = src.W[0];
		dst.W[0].t = t;
		dst.W[1] = w;
		dst.W[1].t = 1.0f-t;
		dst.v = w.p + (src.W[0].p-w.p)*t;
		dst.count=2;
		return 1;
	}

	inline int NextMinkSimplex2(MinkSimplex &dst, const MinkSimplex &src, const MKPoint &w)
	{
		assert(src.count==2);
		const float3 &w0 = src.W[0].p;
		const float3 &w1 = src.W[1].p;
		float t0 = LineProjectTime(w.p ,w0,  float3(0,0,0));
		float t1 = LineProjectTime(w.p ,w1,  float3(0,0,0));
		float3 v0 = w.p + (w0 - w.p) * t0;
		float3 v1 = w.p + (w1 - w.p) * t1;
		int ine0 = (dot(-v0,w1-v0)>0.0f);
		int ine1 = (dot(-v1,w0-v1)>0.0f);

		if(ine0 && ine1)
		{
			dst.count=3;
			float3 v = PlaneProjectOf(w0,w1,w.p,float3(0,0,0));
			dst.v=v;
			dst.W[0] = src.W[0];
			dst.W[1] = src.W[1];
			dst.W[2] = w;
			return 1;
		}
		if(!ine0 && (t0>0.0f)) 
		{
			// keep w0
			dst.W[0] = src.W[0];
			dst.W[0].t = t0;
			dst.W[1] = w;
			dst.W[1].t = 1.0f-t0;
			dst.v = v0 ;
			dst.count=2;
			return 1;
		}
		if(!ine1 && (t1>0.0f))
		{
			// keep w1
			dst.W[0] = src.W[1];
			dst.W[0].t = t1;
			dst.W[1] = w;
			dst.W[1].t = 1.0f-t1;
			dst.v = v1;
			dst.count=2;
			return 1;
		}
		// w by itself is the closest feature
		dst.W[0] = w;
		dst.W[0].t=1.0f;
		dst.v = w.p;
		dst.count=1;
		return 1;
	}

	inline int NextMinkSimplex3(MinkSimplex &dst, const MinkSimplex &src, const MKPoint &w)
	{
		assert(src.count==3);
		const float3 &w0 = src.W[0].p;
		const float3 &w1 = src.W[1].p;
		const float3 &w2 = src.W[2].p;
		float t[3];
		float3 v[3];
		float3 vc[3];
		t[0] = LineProjectTime(w.p ,w0,  float3(0,0,0));
		t[1] = LineProjectTime(w.p ,w1,  float3(0,0,0));
		t[2] = LineProjectTime(w.p ,w2,  float3(0,0,0));
		v[0]  = w.p + (w0 - w.p) * t[0];
		v[1]  = w.p + (w1 - w.p) * t[1];
		v[2]  = w.p + (w2 - w.p) * t[2];
		vc[0] = PlaneProjectOf(w.p,w1,w2, float3(0,0,0));  // project point onto opposing face of tetrahedra
		vc[1] = PlaneProjectOf(w.p,w2,w0, float3(0,0,0));
		vc[2] = PlaneProjectOf(w.p,w0,w1, float3(0,0,0));
		int inp0 = (dot(-vc[0],w0-vc[0])>0.0f) ;
		int inp1 = (dot(-vc[1],w1-vc[1])>0.0f) ;
		int inp2 = (dot(-vc[2],w2-vc[2])>0.0f) ;
	
		// check volume
		if(inp0 && inp1 && inp2)
		{
			// got a  simplex/tetrahedron that contains the origin
			dst=src;
			dst.count=4;
			dst.v= float3(0,0,0);
			dst.W[3]=w;
			return 0;
		}

		// check faces
		int inp2e0 = (dot(-v[0],w1-v[0])>0.0f);
		int inp2e1 = (dot(-v[1],w0-v[1])>0.0f);
		if(!inp2 && inp2e0 && inp2e1)
		{
			dst.count=3;
			float3 v = PlaneProjectOf(w0,w1,w.p,float3(0,0,0));
			dst.v=v;
			dst.W[0] = src.W[0];
			dst.W[1] = src.W[1];
			dst.W[2] = w;
			return 1;
		}
		int inp0e1 = (dot(-v[1],w2-v[1])>0.0f);
		int inp0e2 = (dot(-v[2],w1-v[2])>0.0f);
		if(!inp0 && inp0e1 && inp0e2)
		{
			dst.count=3;
			float3 v = PlaneProjectOf(w1,w2,w.p,float3(0,0,0));
			dst.v=v;
			dst.W[0] = src.W[1];
			dst.W[1] = src.W[2];
			dst.W[2] = w;
			return 1;
		}
		int inp1e2 = (dot(-v[2],w0-v[2])>0.0f);
		int inp1e0 = (dot(-v[0],w2-v[0])>0.0f);
		if(!inp1 && inp1e2 && inp1e0)
		{
			dst.count=3;
			float3 v = PlaneProjectOf(w2,w0,w.p,float3(0,0,0));
			dst.v=v;
			dst.W[0] = src.W[2];
			dst.W[1] = src.W[0];
			dst.W[2] = w;
			return 1;
		}

		// check ridges
		if(!inp1e0 && !inp2e0 && t[0]>0.0f)
		{
			dst.W[0] = src.W[0]; // keep w0
			dst.W[0].t = t[0];
			dst.W[1] = w;
			dst.W[1].t = 1.0f-t[0];
			dst.v = v[0] ;
			dst.count=2;
			return 1;
		}
		if(!inp2e1 && !inp0e1 && t[1]>0.0f)
		{
			dst.W[0] = src.W[1]; // keep w1
			dst.W[0].t = t[1];
			dst.W[1] = w;
			dst.W[1].t = 1.0f-t[1];
			dst.v = v[1] ;
			dst.count=2;
			return 1;
		}
		if(!inp0e2 && !inp1e2 && t[2]>0.0f)
		{
			dst.W[0] = src.W[2]; // keep w2
			dst.W[0].t = t[2];
			dst.W[1] = w;
			dst.W[1].t = 1.0f-t[2];
			dst.v = v[2] ;
			dst.count=2;
			return 1;
		}
		// w by itself is the closest feature
		dst.W[0] = w;
		dst.W[0].t=1.0f;
		dst.v = w.p;
		dst.count=1;
		return 1;

	}

	inline  void fillhitv(Contact &hitinfo,MinkSimplex &src)
	{
		hitinfo.type=-1;
		if(src.count!=3) 
		{
			return;
		}
		if(src.W[0].a == src.W[1].a && src.W[0].a == src.W[2].a )
		{
			// point plane
			hitinfo.type = 1;
			hitinfo.v[0] = src.W[0].a;
			hitinfo.v[1] = src.W[0].b;
			hitinfo.v[2] = src.W[1].b;
			hitinfo.v[3] = src.W[2].b;
			if(dot(hitinfo.normal, cross(src.W[1].p-src.W[0].p,src.W[2].p-src.W[0].p))<0)
			{
				std::swap(hitinfo.v[1],hitinfo.v[2]);
			}
			return;
		}
		if(src.W[0].b == src.W[1].b && src.W[0].b == src.W[2].b )
		{
			// plane point
			hitinfo.type = 3;
			hitinfo.v[0] = src.W[0].a;
			hitinfo.v[1] = src.W[1].a;
			hitinfo.v[2] = src.W[2].a;
			hitinfo.v[3] = src.W[2].b;
			if(dot(hitinfo.normal, cross(src.W[1].p-src.W[0].p,src.W[2].p-src.W[0].p))<0)
			{
				std::swap(hitinfo.v[1], hitinfo.v[2]);
			}
			return;
		}
		if((src.W[0].a == src.W[1].a || src.W[0].a == src.W[2].a || src.W[1].a == src.W[2].a ) && 
		   (src.W[0].b == src.W[1].b || src.W[0].b == src.W[2].b || src.W[1].b == src.W[2].b ))
		{
			// edge edge
			hitinfo.type = 2;
			hitinfo.v[0] = src.W[0].a;
			hitinfo.v[1] = (src.W[1].a!=src.W[0].a)? src.W[1].a : src.W[2].a;
			hitinfo.v[2] = src.W[0].b;
			hitinfo.v[3] = (src.W[1].b!=src.W[0].b)? src.W[1].b : src.W[2].b;
			float dp = dot(hitinfo.normal, cross(src.W[1].p-src.W[0].p,src.W[2].p-src.W[0].p));
			if((dp<0 && src.W[1].a!=src.W[0].a)||(dp>0 && src.W[1].a==src.W[0].a))
			{
				std::swap(hitinfo.v[2], hitinfo.v[3]);
			}
			return;
		}
		if(src.W[0].b != src.W[1].b && src.W[0].b != src.W[2].b && src.W[1].b != src.W[2].b )
		{
		}
		if(src.W[0].a != src.W[1].a && src.W[0].a != src.W[2].a && src.W[1].a != src.W[2].a )
		{
		}
		// could be point point or point edge
		// therefore couldn't classify type to something that suggests a plane from available vertex indices.
	}
	inline Contact calcpoints(std::function<float3(const float3&)> A, std::function<float3(const float3&)>B, MinkSimplex &src)  // Contact returned will be 'true'
	{
		int i;
		assert(src.count>0);
		if(src.count==3) // need to computed weights t
		{
			float3 b = BaryCentric(src.W[0].p,src.W[1].p,src.W[2].p,src.v);
			src.W[0].t=b.x;
			src.W[1].t=b.y;
			src.W[2].t=b.z;
		}
		src.pa=src.pb=float3(0,0,0);
		for(i=0;i<src.count;i++) 
		{
			src.pa += src.W[i].t * src.W[i].a;
			src.pb += src.W[i].t * src.W[i].b;
		}
		Contact hitinfo;
		hitinfo.p0w = src.pa;
		hitinfo.p1w = src.pb;
		hitinfo.impact = (src.pa + src.pb)*0.5f;
		hitinfo.separation = magnitude(src.pa - src.pb )+ FLT_MIN;
		hitinfo.normal     =  normalize(src.v);
		hitinfo.dist       = -dot(hitinfo.normal,hitinfo.impact);
		fillhitv(hitinfo,src);
		return hitinfo;   // hitinfo evaluates to 'true' since the separation is > 0.
	}

	class MTri
	{
	public: 
		int w[3];
		float3 v;
		MTri(){w[0]=w[1]=w[2]=-1;}
		MTri(int w0,int w1,int w2,const float3 &_v){w[0]=w0;w[1]=w1;w[2]=w2;v=_v;}
		MTri(int w0,int w1,int w2,const std::vector<MKPoint> &W){w[0]=w0;w[1]=w1;w[2]=w2;
															 v=PlaneProjectOf(W[w0].p,W[w1].p,W[w2].p,float3(0,0,0));
								float3 b = BaryCentric(W[w0].p,W[w1].p,W[w2].p,v);
									if(b.x<0 || b.y<0 || b.z<0 || b.x>1.0f || b.y>1.0f || b.z>1.0f) v=(W[w0].p + W[w1].p+W[w2].p)/3.0f;}
	};

	class Expandable
	{
	 public:
		std::vector<MKPoint> W;
		std::vector<MTri> tris;
		void expandit(MinkSimplex &dst, std::function<float3(const float3&)> A, std::function<float3(const float3&)>B);
		Expandable(const MinkSimplex &src);
	};

	inline Expandable::Expandable(const MinkSimplex &src)
	{
		int i;
		assert(src.count==4);
		for(i=0;i<4;i++)
		{
			W.push_back(src.W[i]);
		}
		tris.push_back(MTri(0,1,2,W));
		tris.push_back(MTri(1,0,3,W));
		tris.push_back(MTri(2,1,3,W));
		tris.push_back(MTri(0,2,3,W));
	}

	inline void Expandable::expandit(MinkSimplex &dst, std::function<float3(const float3&)> A, std::function<float3(const float3&)>B )
	{
		float prevdist = 0.0f;
		int done=0;
		while(1)
		{
			int closest=0;
			for(unsigned i=0;i<tris.size();i++)
			{
				if(magnitude(tris[i].v)<magnitude(tris[closest].v)) 
				{
					closest=i;
				}
			}
			float3 v = tris[closest].v;
			if(dot(v,v) <= prevdist)
			{
				done=1;
			}
			prevdist = dot(v,v);
			MKPoint w = PointOnMinkowski(A,B,v); 
			if(w== W[tris[closest].w[0]] ||w== W[tris[closest].w[1]] ||w== W[tris[closest].w[2]] )
			{
				done=1;
			}
			if(done ||  dot(w.p,v) < dot(v,v) + 0.0001) {
				//done
				dst.count=3;
				dst.v = v;
				dst.W[0] = W[tris[closest].w[0]];
				dst.W[1] = W[tris[closest].w[1]];
				dst.W[2] = W[tris[closest].w[2]];
				return;
			}
			int wi=W.size();
			W.push_back(w);
			tris.push_back(MTri(tris[closest].w[0],tris[closest].w[1],wi, W ));
			tris.push_back(MTri(tris[closest].w[1],tris[closest].w[2],wi, W ));
			tris.push_back(MTri(tris[closest].w[2],tris[closest].w[0],wi, W ));
			tris.erase(begin(tris) + closest);
		}
	}


	inline Contact Separated(std::function<float3(const float3&)> A, std::function<float3(const float3&)>B, int findclosest)
	{
		int(*NextMinkSimplex[4])(MinkSimplex &dst, const MinkSimplex &src, const MKPoint &w) =
		{
			NextMinkSimplex0,
			NextMinkSimplex1,
			NextMinkSimplex2,
			NextMinkSimplex3
		};
		MinkSimplex last;
		MinkSimplex next;
		int iter=0;
		float3 v = PointOnMinkowski(A,B,float3(0,0,1)).p;
		last.count=0;
		last.v=v;
		MKPoint w = PointOnMinkowski(A,B,-v);
		NextMinkSimplex[0](next,last,w);
		last=next;
		// todo: add the use of the lower bound distance for termination conditions
		while((dot(w.p,v) < dot(v,v) - 0.00001f) && iter++<100) 
		{
			int isseparated;
			last=next;  // not ideal, a swapbuffer would be better
			v=last.v;
			w=PointOnMinkowski(A,B,-v);
			if(dot(w.p,v) >= dot(v,v)- 0.00001f - 0.00001f*dot(v,v))
			{
				//  not getting any closer here
				break;
			}
			if(!findclosest  &&  dot(w.p,v)>=0.0f) // found a separating plane
			{
				break;
			}
			isseparated = NextMinkSimplex[last.count](next,last,w);
			if(!isseparated)
			{
				Expandable expd(next);
				expd.expandit(last,A,B);
				float4 separationplane(0,0,0,0);
				separationplane.xyz() = TriNormal(last.W[0].p,last.W[1].p,last.W[2].p);
				if(dot(separationplane.xyz(),last.v)>0) separationplane.xyz() *=-1.0f;
				Contact hitinfo;
				hitinfo.normal = separationplane.xyz();
				hitinfo.dist   = separationplane.w;
				float3 b = BaryCentric(last.W[0].p,last.W[1].p,last.W[2].p,last.v);
				last.W[0].t=b.x;
				last.W[1].t=b.y;
				last.W[2].t=b.z;
				last.pa = b.x * last.W[0].a + b.y * last.W[1].a + b.z*last.W[2].a;
				last.pb = b.x * last.W[0].b + b.y * last.W[1].b + b.z*last.W[2].b;
				hitinfo.p0w   =  last.pa;
				hitinfo.p1w   =  last.pb;
				hitinfo.impact=  (last.pa+last.pb )*0.5f;
				separationplane.w = -dot(separationplane.xyz(), hitinfo.impact);
				hitinfo.separation = -magnitude(last.pa - last.pb); //  dot(separationplane.normal,w.p-v);
			 
				fillhitv(hitinfo,last);
				assert(hitinfo);
				return hitinfo;
			}
			if(dot(next.v,next.v)>=dot(last.v, last.v))   // i.e. if magnitude(w.p)>magnitude(v) 
			{
				break;  // numerical screw up, 
			}
		}
		assert(iter<100);
		return calcpoints(A,B,last);
	}


	inline  Contact tunnel(std::function<float3(const float3&)> A, std::function<float3(const float3&)>B, const float3& ray, MinkSimplex &start)
	{
		int tet[4][3] = {{0,1,2},{1,0,3},{2,1,3},{0,2,3}};
		int i;
		MKPoint v0,v1,v2;
		int k=0;
		int s=-1;
		for(i=0;i<4;i++)
		{
			if(tri_interior(start.W[tet[i][0]].p,start.W[tet[i][1]].p,start.W[tet[i][2]].p,ray))
			{
				s=i;
				k++;
				v0 = start.W[tet[i][0]];
				v1 = start.W[tet[i][1]];
				v2 = start.W[tet[i][2]];
			}
		}
		assert(k==1);
		assert(s>=0);
		float3 n = TriNormal(v0.p, v1.p, v2.p);
		if(dot(n,ray)<0.0f)
		{
			n=-n;
			MKPoint tmp;
			tmp=v0;
			v0=v1;
			v1=tmp;
		}
		MKPoint v = PointOnMinkowski(A,B,ray,n);

		int iterlimit=0;
		while(iterlimit++<100 && v.p!=v0.p && v.p!= v1.p && v.p!=v2.p)
		{
			if(tri_interior(v0.p,v1.p,v.p,ray))
			{
				v2=v;
			}
			else if(tri_interior(v1.p,v2.p,v.p,ray))
			{
				v0=v;
			}
			else 
			{
				assert(tri_interior(v2.p,v0.p,v.p,ray));
				v1=v;
			}
			n = TriNormal(v0.p, v1.p, v2.p);
			v = PointOnMinkowski(A,B,ray,n);
			//v = mkp.p;
		}
		Contact hitinfo;
		hitinfo.normal = -n;
		if(dot(hitinfo.normal,ray) > 0.0f)
		{
			hitinfo.normal *= -1.0f;  // i dont think this should happen, v0v1v2 winding should have been consistent
		}
		hitinfo.dist   = -dot(hitinfo.normal,v.p);
		float3 hitpoint = PlaneLineIntersection(hitinfo.normal,hitinfo.dist,float3(0,0,0),ray);
		float time = 1.0f - sqrtf(dot(hitpoint,hitpoint)/dot(ray,ray));
		hitinfo.time = time;
		float3 b = BaryCentric(v0.p,v1.p,v2.p, hitpoint);
		hitinfo.impact =  b.x * v0.b + b.y * v1.b + b.z*v2.b;
		hitinfo.separation = time-1.0f ; // not sure yet what to put here other than negative.
		return hitinfo;
	}

	inline Contact Sweep(std::function<float3(const float3&)>A, std::function<float3(const float3&)>B, const float3& dir)
	{
		// For this moving version, Jay Stelly gets credit for the idea to tunnel in the reverse direction along the ray
		int(*NextMinkSimplex[4])(MinkSimplex &dst, const MinkSimplex &src, const MKPoint &w) =
		{
			NextMinkSimplex0,
			NextMinkSimplex1,
			NextMinkSimplex2,
			NextMinkSimplex3
		};
		Contact hitinfo;
		MinkSimplex last;
		MinkSimplex next;
		int iter=0;
		float3 v = PointOnMinkowski(A,B,dir,float3(0,0,1)).p;
		last.count=0;
		last.v=v;
		MKPoint w = PointOnMinkowski(A,B,dir,-v);
		NextMinkSimplex[0](next,last,w);
		last=next;
		// todo: add the use of the lower bound distance for termination conditions
		while((dot(w.p,v) < dot(v,v) - 0.00001f) && iter++<100) 
		{
			int isseparated;
			last=next;  // not ideal, a swapbuffer would be better
			v=last.v;
			w=PointOnMinkowski(A,B,dir,-v);
			if(dot(w.p,v) >= dot(v,v)- 0.00001f - 0.00001f*dot(v,v))
			{
				//  not getting any closer here
				break;
			}
			if(dot(w.p,v)>=0.0f) // found a separating plane  // && !findclosest  
			{
				break;
			}
			isseparated = NextMinkSimplex[last.count](next,last,w);
			if(!isseparated)
			{
				// tunnel back and find the 
				return tunnel(A, B, dir, next);
			}
			if(dot(next.v,next.v)>=dot(last.v, last.v))   // i.e. if length(w.p)>length(v) 
			{
				break;  // numerical screw up, 
			}
		}
		assert(iter<100);
		return calcpoints(A,B,last);  // true  (separated)
	}

} // namespace  gjk_implementation


inline gjk_implementation::Contact Separated(std::function<float3(const float3&)> A, std::function<float3(const float3&)>B, int findclosest=1)
{
	return gjk_implementation::Separated(A, B, findclosest);  // pass through into the namespace  
}

inline std::function<float3(const float3&)> SupportFunc(const std::vector<float3> &points)   // example of how one might write an on-the-fly (lambda) support function for a container
{
	return[&points](const float3 &dir){ return points[maxdir(points.data(), points.size(), dir)]; };
}
inline gjk_implementation::Contact Separated(const std::vector<float3> &a, const std::vector<float3> &b, int findclosest=1)  // how to call Separated for two point clouds
{
	return gjk_implementation::Separated(SupportFunc(a), SupportFunc(b), findclosest);
}


inline std::function<float3(const float3&)> SupportFuncTrans(std::function<float3(const float3&)> sf, const float3& position, const float4 &orientation)   // example supportfunc for posed meshes 
{
	return [sf, position, orientation](const float3 &dir) {return position + qrot(orientation, sf(qrot(qconj(orientation), dir))); };
}


inline gjk_implementation::Contact Separated(const std::vector<float3> &a,const float3 &ap,const float4 &aq, const std::vector<float3> &b,const float3 &bp,const float4 &bq, int findclosest=1)
{
	return gjk_implementation::Separated(SupportFuncTrans(SupportFunc(a), ap, aq), SupportFuncTrans(SupportFunc(b), bp, bq), findclosest);
}

//------ Contact Patch -----------------



struct Patch
{
	gjk_implementation::Contact hitinfo[5];
	int count;
	gjk_implementation::Contact &operator[](int i){ return hitinfo[i]; }
	gjk_implementation::Contact *begin(){ return hitinfo; }
	gjk_implementation::Contact *end(){ return hitinfo + count; }
	operator bool(){ return (count != 0); }
	Patch() :count(0){}
};


template<class CA, class CB>
inline Patch ContactPatch(CA s0, CB s1, float max_separation)  // return 0 if s1 and s1 separated more than max_separation
{
	// This routine rolls one of the inputs on the axes parallel to the separating plane to approximate the contact area 
	// using the gjk's separated convention right now   points from s1 to s0   would have prefered point from s0 to s1
	Patch hitinfo;

	hitinfo[0] = Separated(s0, s1, 1);

	if (hitinfo[0].separation>max_separation)
		return hitinfo;  // too far away to care

	const float3 &n = hitinfo[0].normal;  // since this needs to be flipped than what was in my head today
	int &hc = ++hitinfo.count;   // set count to 1, including initial sample in patch
	float3 tangent = Orth(n);
	float3 bitangent = cross(n, tangent); // tangent X bitangent == n
	float3 rollaxes[4] = { tangent, bitangent, -tangent, -bitangent };
	for (auto &raxis : rollaxes)
	{
		const float contactpatchjiggle = 2.0f;// rotations in degrees   ideally this should be a shear
		float4 jiggle = normalize(float4(raxis*sinf(3.14f / 180.0f*(contactpatchjiggle)), 1));
		float3 pivot = hitinfo[0].p0w;
		Pose ar = Pose(n*0.1f, float4(0, 0, 0, 1))  * Pose(-pivot, float4(0, 0, 0, 1)) * Pose(float3(0, 0, 0), jiggle) * Pose(pivot, float4(0, 0, 0, 1));
		hitinfo[hc] = Separated(SupportFuncTrans(s0, ar.position, ar.orientation), s1, 1);
		hitinfo[hc].normal = n;// all parallel to the initial separating plane;
		hitinfo[hc].p0w = ar.Inverse() * hitinfo[hc].p0w;  // contact back into unadjusted space
		bool match = false;
		for (int j = 0; !match && j<hc; j++)
			match = magnitude(hitinfo[hc].p0w - hitinfo[j].p0w) < 0.05f || magnitude(hitinfo[hc].p1w - hitinfo[j].p1w) < 0.05f;
		if (match)
			continue;
		hc++;
	}
	return hitinfo;
}



#endif // GJK_H
