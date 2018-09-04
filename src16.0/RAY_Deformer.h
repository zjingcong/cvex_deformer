
#ifndef __RAY_Deformer__
#define __RAY_Deformer__

#include <UT/UT_BoundingBox.h>
#include <UT/UT_String.h>
#include <UT/UT_ThreadedAlgorithm.h>
#include <UT/UT_Lock.h>
#include <UT/UT_PtrArray.h>
#include <UT/UT_TaskScope.h>
#include <UT/UT_Thread.h>
#include <UT/UT_Array.h>
#include <UT/UT_Interrupt.h>

#include <VRAY/VRAY_Procedural.h>
#include <VRAY/VRAY_IO.h>
#include <VRAY/VRAY_ProceduralFactory.h>

#include <CVEX/CVEX_Context.h>
#include <CVEX/CVEX_Value.h>
#include <CVEX/CVEX_ValueList.h>
#include <VEX/VEX_Error.h>
#include <GVEX/GVEX_GeoCommand.h>

#include <GA/GA_Handle.h>

#include <GU/GU_Detail.h>
#include <GEO/GEO_Point.h>
#include <GEO/GEO_Vertex.h>
#include <GEO/GEO_Primitive.h>
#include <GEO/GEO_AttributeHandle.h>

#include <GU/GU_PolyFrame.h>

#include <unordered_map>
#include <algorithm>

#define CHUCK_SIZE	1024
#define CVEX_MAX_NUM	4

// match those with Houdini Deformer parm interface
#define DO_POINTS	0
#define DO_PRIMS	1
#define DO_VERTS	2
#define DO_DETAILS	3

namespace HDK_Deform
{

	class RAY_Deform;

	struct CVEXProcessData
	{
		RAY_Deform* instance_pt;
		GU_Detail *gd;
		VEX_GeoCommandQueue** threadcmds;
		int gid;
		int tid;
		int total_size;
		int threadnum;
		fpreal32 shutter;

		CVEXProcessData(RAY_Deform* pt, GU_Detail* gd, VEX_GeoCommandQueue** cmds, int tid, int tsize, int num, fpreal32 shut) :
			instance_pt(pt),
			gd(gd),
			threadcmds(cmds),
			tid(tid),
			total_size(tsize),
			threadnum(num),
			shutter(shut)
		{}
	};

	struct CVEXExtraAttribMap
	{
		std::unordered_map<UT_StringHolder, UT_Vector3> vec3AttribMap;
		std::unordered_map<UT_StringHolder, fpreal32> floatAttribMap;
		std::unordered_map<UT_StringHolder, UT_Vector4> vec4AttribMap;
		std::unordered_map<UT_StringHolder, int> intAttribMap;
	};

	class RAY_Deform : public VRAY_Procedural
	{
	public:
		RAY_Deform(UT_Vector3 center, UT_StringHolder infile, 
			std::vector<UT_StringHolder>& cfiles, CVEXExtraAttribMap& cextra, 
			std::vector<int>& cruntypes, int cvexn, int isMultiT, int ins,
			int compN, int isVB, int geoTSample, fpreal open, fpreal close, fpreal fps, 
			int* polyframeflags, const GU_PolyFrameParms& pf_parms);
		virtual ~RAY_Deform();
		virtual const char *className() const;
		virtual int initialize(const UT_BoundingBox *);
		virtual void getBoundingBox(UT_BoundingBox &box);
		virtual void render();

	private:
		bool isSuccess;	// success status for this procedural preprocessing
		/// geom boundingbox
		UT_BoundingBox bbox;
		/// input parms
		UT_Vector3 center_pos;
		UT_StringHolder inputfile;
		std::vector<UT_StringHolder>& cvexfiles;
		CVEXExtraAttribMap& cvex_extraAttribs;
		std::vector<int>& cvex_runtypes;
		int instance_id;
		int cvex_num;
		int is_multi_threads;
		int is_compute_normal;
		int is_velBlur;
		int geo_timeSample;
		fpreal camShutter_open;
		fpreal camShutter_close;
		fpreal fps;
		int* polyframe_flags;
		const GU_PolyFrameParms& polyframe_parms;
		/// cvex files and run types
		UT_StringHolder inputcvex;
		int cvex_runtype;	// cvextype: 0-points, 1-primitives, 2-vertices
		/// input geom
		VRAY_ProceduralGeo geo;
		/// cvex parms
		// attributes list
		std::vector<GA_Attribute*> geoattriblist;
		std::vector<UT_StringHolder> cvexoutputnamelist;
		// cvex input list: buffer to store input for each input attrib
		std::vector<std::vector<void*>> inputbuffer;	// used for handle memory allocation
		// cvex output list: buffer to store output for each output attrib
		template <class T>
		using AttribMapT = std::unordered_map<UT_StringHolder, T*>;
		std::vector<AttribMapT<UT_Vector3>> vec3_outputbuffer;		// cvextype: CVEX_TYPE_VECTOR3
		std::vector<AttribMapT<fpreal32>> float_outputbuffer;		// cvextype: CVEX_TYPE_FLOAT
		std::vector<AttribMapT<UT_Vector4>> vec4_outputbuffer;		// cvextype: CVEX_TYPE_VECTOR4
		std::vector<AttribMapT<int>> int_outputbuffer;				// cvextype: CVEX_TYPE_INTEGER

		UT_Lock theLock;

		int preprocess();
		void cleanBuffer();
		
		bool loadGeo();
		void velBBox(GU_Detail *gd, UT_BoundingBox& box);
		void polyFrame(GU_Detail *gd);

		/// cvex
		void executeCVEX(GU_Detail *gd, fpreal32 shutter);
		// cvex processing
		bool processCVEX(CVEX_Context &context, CVEX_RunData &rundata, GU_Detail *gd, int gid, int size, int tid, fpreal32 shutter);
		// get geom attributes
		void getGeomAttribs(GU_Detail *gd);
		// set cvex function inputs from geom attributes
		void addCVEXInput(CVEX_Context &context);
		// load cvex function
		bool loadCVEX(CVEX_Context &context);
		// find cvex function inputs and outputs, allocate memory for output results
		void findCVEX(CVEX_Context &context, GU_Detail *gd, int gid, int size, int tid, fpreal32 shutter);
		void setCVEXOutput(CVEX_Context &context, GU_Detail *gd, int gid, int size, int tid);
		// create new output
		void createGeomAttribFromCVEXOutput(CVEX_Context &context, GU_Detail *gd);

		// process CVEX separately in thread
		static void* executeSingleCVEX(void* data)
		{
			// depack all the input data
			CVEXProcessData* input = reinterpret_cast<CVEXProcessData*>(data);
			RAY_Deform* instance_pt = input->instance_pt;
			GU_Detail* gd = input->gd;
			VEX_GeoCommandQueue** threadcmds = input->threadcmds;
			int tid = input->tid;
			int thread_num = input->threadnum;
			int total_size = input->total_size;
			fpreal32 shutter = input->shutter;

			// init
			CVEX_Context cvex;
			CVEX_RunData rundata;

			// set gvex queue
			UT_Array<exint> procid(CHUCK_SIZE, CHUCK_SIZE);
			rundata.setProcId(procid.array());
			VEX_GeoCommandQueue* geocmd = new VEX_GeoCommandQueue();
			geocmd->myNumPrim = gd->getNumPrimitives();
			geocmd->myNumVertex = gd->getNumVertices();
			geocmd->myNumPoint = gd->getNumPoints();
			rundata.setGeoCommandQueue(geocmd);
			threadcmds[tid] = geocmd;

			int size = 0;
			int gid = tid * CHUCK_SIZE;
			// set procid with prim/point/vertex id
			for (int i = 0; i < CHUCK_SIZE; ++i)	{ procid(i) = gid + i;}
			// set buffer size
			if (tid < thread_num - 1)	{ size = CHUCK_SIZE;}
			else						{ size = total_size % CHUCK_SIZE;}
			// run cvex processing
			{
				// UT_Lock::Scope lock(instance_pt->theLock);
				instance_pt->addCVEXInput(cvex);
				// load cvex
				if (!instance_pt->loadCVEX(cvex)) { return false; }
				// allocate memory for input and output
				instance_pt->findCVEX(cvex, gd, gid, size, tid, shutter);
				// run cvex program
				cvex.run(size, true, &rundata);
				// pass cvex result back to geom
				instance_pt->setCVEXOutput(cvex, gd, gid, size, tid);
			}

			return nullptr;
		}

		// cvex find typed input and output
		template <typename T>
		inline void findTypedUniformInput(CVEX_Context &context, UT_StringHolder name, T* value)
		{
			CVEX_Value* var;
			CVEX_Type type = type2CVEXTypeHandler<T>();
			var = context.findInput(name, type);
			if (var)
			{
				var->setTypedData(value, 1);
			}
		}

		template <typename T>
		inline void findTypedCVEX(CVEX_Context &context, GU_Detail *gd, std::vector<AttribMapT<T>>& attriblist, int gid, int size, int tid)
		{
			GA_AttributeOwner owner;
			GA_Offset off;
			switch (cvex_runtype)
			{
			case DO_POINTS:
			{
				off = gd->pointOffset(GA_Index(gid));
				owner = GA_ATTRIB_POINT;
				break;
			}
			case DO_PRIMS:
			{
				off = gd->primitiveOffset(GA_Index(gid));
				owner = GA_ATTRIB_PRIMITIVE;
				break;
			}
			case DO_VERTS:
			{
				off = gd->vertexOffset(GA_Index(gid));
				owner = GA_ATTRIB_VERTEX;
				break;
			}
			case DO_DETAILS:
			{
				off = 0;
				owner = GA_ATTRIB_DETAIL;
				break;
			}
			default:
				VRAYerrorOnce("CVEX %s as runtype %d: Cannot identify CVEX run type.", inputcvex.c_str(), cvex_runtype);
				return;
			}

			CVEX_Value* val;
			CVEX_Value* out;
			CVEX_Type type = type2CVEXTypeHandler<T>();

			T* attr_list;
			T* attr_outlist;

			for (auto attrib : geoattriblist)
			{
				// check to see whether VEX function has the correspoonding parameter
				val = context.findInput(attrib->getName(), type);
				// if input, allocate memory and set input data
				if (val)
				{
					// allocate memory for input attrib
					attr_list = new T[size];
					inputbuffer[tid].push_back((void*)attr_list);
					// set attrib to buffer
					getTypedAttribByGeom(gd, attrib->getName(), attr_list, owner, off, size);
					// set cvex input
					val->setTypedData(attr_list, size);
					// RAYprintf(0, "Set input attrib %s.", attrib->getName().c_str());
				}
			}

			for (auto name : cvexoutputnamelist)
			{
				// find output of cvex
				out = context.findOutput(name, type);
				if (out)
				{
					// allocate memory for output list
					attr_outlist = new T[size];
					attriblist[tid][name] = attr_outlist;
					// link memory buffer to output attrib
					out->setTypedData(attr_outlist, size);
					// RAYprintf(0, "Set output attrib %s.", name.c_str());
				}
			}
		}

		// set cvex typed output back to geom attributes
		template <typename T>
		inline void setTypedCVEXOutput(CVEX_Context &context, GU_Detail *gd, std::vector<AttribMapT<T>>& attriblist, int gid, int size, int tid)
		{
			GA_AttributeOwner owner;
			GA_Offset off;
			switch (cvex_runtype)
			{
			case DO_POINTS:
			{
				off = gd->pointOffset(GA_Index(gid));
				owner = GA_ATTRIB_POINT;
				break;
			}
			case DO_PRIMS:
			{
				off = gd->primitiveOffset(GA_Index(gid));
				owner = GA_ATTRIB_PRIMITIVE;
				break;
			}
			case DO_VERTS:
			{
				off = gd->vertexOffset(GA_Index(gid));
				owner = GA_ATTRIB_VERTEX;
				break;
			}
			case DO_DETAILS:
			{
				off = 0;
				owner = GA_ATTRIB_DETAIL;
				break;
			}
			default:
				VRAYerrorOnce("CVEX %s as runtype %d: Cannot identify CVEX run type.", inputcvex.c_str(), cvex_runtype);
				return;
			}

			for (const auto &attribinfo : attriblist[tid])
			{
				CVEX_Value* out;
				UT_StringHolder attrib_nam = attribinfo.first;
				T* outputlist = attribinfo.second;
				out = context.findOutput(attrib_nam, type2CVEXTypeHandler<T>());
				setTypedAttribByGeom<T>(gd, attrib_nam, outputlist, owner, off, size);
				// RAYprintf(0, "Set attrib %s back to geom.", attrib_nam.c_str());
			}
		}

		// get typed attributes data from input geom
		template <typename T>
		inline bool getTypedAttribByGeom(GU_Detail *gd, UT_StringHolder name, T* inputlist, GA_AttributeOwner owner, GA_Offset off, int size)
		{
			GA_ROHandleT<T> handle(gd, owner, name);
			if (handle.isValid())
			{
				T attr;
				for (int i = 0; i < size; ++i)
				{
					GA_Offset offset = off + i;
					attr = handle.get(offset);
					inputlist[i] = attr;
				}
				return true;
			}
			else
			{
				VRAYwarningOnce("CVEX %s as runtype %d: Handle for attribute %s is not valid.", inputcvex.c_str(), cvex_runtype, name.c_str());
				return false;
			}
		}

		// set typed attributes data to input geom
		template <typename T>
		inline bool setTypedAttribByGeom(GU_Detail *gd, UT_StringHolder name, T* outputlist, GA_AttributeOwner owner, GA_Offset off, int size)
		{
			GA_RWHandleT<T> handle(gd, owner, name);
			if (handle.isValid())
			{
				for (int i = 0; i < size; ++i)
				{
					GA_Offset offset = off + i;
					handle.set(offset, (const T)outputlist[i]);
				}
				return true;
			}
			else
			{
				VRAYwarningOnce("CVEX %s as runtype %d: Handle for attribute %s is not valid.", inputcvex.c_str(), cvex_runtype, name.c_str());
				return false;
			}
		}

		/// type handlers
		// type handler matching geo attrib to cvex type
		CVEX_Type attrib2CVEXTypeHandler(GA_Attribute* attrib);

		// type handler matching hdk type to cvex type
		template <typename T>
		inline CVEX_Type type2CVEXTypeHandler()
		{
			if (std::is_same<T, UT_Vector3>::value) { return CVEX_TYPE_VECTOR3; }
			if (std::is_same<T, fpreal32>::value) { return CVEX_TYPE_FLOAT; }
			if (std::is_same<T, UT_Vector4>::value) { return CVEX_TYPE_VECTOR4; }
			if (std::is_same<T, int>::value) { return CVEX_TYPE_INTEGER; }
			return CVEX_TYPE_INVALID;
		}
	};

}

#endif
