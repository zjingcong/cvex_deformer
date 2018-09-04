
#include "RAY_DeformInstance.h"
#include <UT/UT_DSOVersion.h>

// point cloud
#define PC_INSTANCEFILE_ATTRIB "instancefile"
#define PC_ATTRIB_PREFIX "point_"

using namespace HDK_Deform;

//** register procedural

// args
static VRAY_ProceduralArg        theArgs[] = {
	VRAY_ProceduralArg("file", "string", ""),
	VRAY_ProceduralArg("loadPointCloud", "int", "0"),
	VRAY_ProceduralArg("instance", "int", "0"),
	VRAY_ProceduralArg("computeN", "int", "0"),

	/// motion blur
	VRAY_ProceduralArg("velBlur", "int", "0"),
	VRAY_ProceduralArg("geoTimeSample", "int", "1"),

	/// cvex
	VRAY_ProceduralArg("cvexnum", "int", "0"),
	VRAY_ProceduralArg("isMultiThreads", "int", "0"),
	// cvex list
	VRAY_ProceduralArg("CVEX1", "string", ""),
	VRAY_ProceduralArg("CVEX2", "string", ""),
	VRAY_ProceduralArg("CVEX3", "string", ""),
	VRAY_ProceduralArg("CVEX4", "string", ""),
	VRAY_ProceduralArg("CVEX_type1", "int", "0"),
	VRAY_ProceduralArg("CVEX_type2", "int", "0"),
	VRAY_ProceduralArg("CVEX_type3", "int", "0"),
	VRAY_ProceduralArg("CVEX_type4", "int", "0"),

	/// polyframe
	VRAY_ProceduralArg("prePolyframe", "int", "0"),
	VRAY_ProceduralArg("postPolyframe1", "int", "0"),
	VRAY_ProceduralArg("postPolyframe2", "int", "0"),
	VRAY_ProceduralArg("postPolyframe3", "int", "0"),
	VRAY_ProceduralArg("postPolyframe4", "int", "0"),
	// polyframe setup
	VRAY_ProceduralArg("style", "int", "0"),
	VRAY_ProceduralArg("which", "int", "1"),
	VRAY_ProceduralArg("normName", "string", "N"),
	VRAY_ProceduralArg("tanName", "string", "tangentu"),
	VRAY_ProceduralArg("bitanName", "string", "tangentv"),
	VRAY_ProceduralArg("orthogonal", "int", "0"),
	VRAY_ProceduralArg("leftHanded", "int", "0"),
	VRAY_ProceduralArg("uvName", "string", ""),

	VRAY_ProceduralArg()
};

class ProcDef : public VRAY_ProceduralFactory::ProcDefinition
{
public:
	ProcDef()
		: VRAY_ProceduralFactory::ProcDefinition("deformer")
	{
	}
	virtual VRAY_Procedural      *create() const { return new RAY_DeformInstance(); }
	virtual VRAY_ProceduralArg   *arguments() const { return theArgs; }
};

void registerProcedural(VRAY_ProceduralFactory *factory)
{
	factory->insert(new ProcDef);
}

/// --------------------------------------------------------------------------------------------------------

//** parent procedural: deformer instance

RAY_DeformInstance::RAY_DeformInstance()
{
	bbox.initBounds(0.0, 0.0, 0.0);
}

RAY_DeformInstance::~RAY_DeformInstance()
{
	//! when delete those memory, each time adjusting parm in deformer without stoping mantra, mantra will crash
	//for (auto deformer_pt : childDeformer_list)
	//{
	//	delete deformer_pt;
	//}
	//for (auto attribmap : attribmaps)
	//{
	//	delete attribmap;
	//}
}

const char* RAY_DeformInstance::className() const
{
	return "RAY_DeformInstance";
}

int RAY_DeformInstance::initialize(const UT_BoundingBox *box)
{
	VRAYprintf(0, "=== InstanceDeformer Init ===\n");

	/// input parms
	UT_StringHolder inputfile;
	int is_pCloud;
	int instancenum;
	int is_compute_normal;
	int is_velBlur;
	int geo_timeSample;
	int cvexnum;
	int isMultiThreads;
	/// cvex list
	std::vector<UT_StringHolder> cvexfiles;
	std::vector<int> cvex_runtypes;
	/// polyframe
	int polyframe_flags[5];	// pre_polyframe, post_polyframe1, post_polyframe2, post_polyframe3, post_polyframe4

	/// load deformer parms
	// load argvs
	import("file", inputfile);
	import("loadPointCloud", &is_pCloud, 1);
	import("instance", &instancenum, 1);
	import("computeN", &is_compute_normal, 1);
	import("cvexnum", &cvexnum, 1);
	import("isMultiThreads", &isMultiThreads, 1);
	import("velBlur", &is_velBlur, 1);
	import("geoTimeSample", &geo_timeSample, 1);
	fpreal fps = 24.0, camshutter[2] = { 0 };
	import("global:fps", &fps, 1);
	import("camera:shutter", camshutter, 2);
	import("prePolyframe", &polyframe_flags[0], 1);
	import("postPolyframe1", &polyframe_flags[1], 1);
	import("postPolyframe2", &polyframe_flags[2], 1);
	import("postPolyframe3", &polyframe_flags[3], 1);
	import("postPolyframe4", &polyframe_flags[4], 1);
	VRAYprintf(0, "Load parm: \n\tfile: %s\n\tis_pCloud: %d\n\tinstance: %d\n\tcvexnum: %d\n\tisMultiThreads: %d\n\tcomputeN: %d\n\tvelBlur: %d\n\tgeoTimeSample: %d\n\tFPS: %f\n\tcamShutter: %f, %f",
		inputfile.c_str(), is_pCloud, instancenum, cvexnum, isMultiThreads, is_compute_normal, is_velBlur, geo_timeSample, fps, camshutter[0], camshutter[1]);
	VRAYprintf(0, "Load polyframe enable info: \n\tprePolyframe: %d\n\tpostPolyframe1: %d\n\tpostPolyframe2: %d\n\tpostPolyframe3: %d\n\tpostPolyframe4: %d",
		polyframe_flags[0], polyframe_flags[1], polyframe_flags[2], polyframe_flags[3], polyframe_flags[4]);
	
	// load polyframeParms
	GU_PolyFrameParms polyframe_parms;
	if (polyframe_flags[0] || polyframe_flags[1] || polyframe_flags[2] || polyframe_flags[3] || polyframe_flags[4])
	{
		import("which", &polyframe_parms.which, 1);
		UT_StringHolder N_name, tangentu_name, tangentv_name;
		import("normName", N_name);
		import("tanName", tangentu_name);
		import("bitanName", tangentv_name);
		polyframe_parms.names[0] = tangentu_name.c_str();
		polyframe_parms.names[1] = tangentv_name.c_str();
		polyframe_parms.names[2] = N_name.c_str();
		int orthogonal, left_handed;
		import("orthogonal", &orthogonal, 1);
		import("leftHanded", &left_handed, 1);
		polyframe_parms.orthogonal = (bool)orthogonal;
		// polyframe_parms.left_handed = (bool)left_handed;     /// hdk16.5
		int style;
		import("style", &style, 1);
		GU_PolyFrameStyle styleHandler[5] = 
		{ GU_POLYFRAME_FIRST_EDGE, 
			GU_POLYFRAME_TWO_EDGES, 
			GU_POLYFRAME_CENTROID, 
			GU_POLYFRAME_TEXTURE_COORDS, 
			GU_POLYFRAME_TEXTURE };
		polyframe_parms.style = styleHandler[style];
		UT_StringHolder uv_name;
		import("uvName", uv_name);
		polyframe_parms.uv_name = uv_name.c_str();
		VRAYprintf(0, "Load polyframe parm: \n\twhich: %d\n\tnormName: %s, tanName: %s, bitanName: %s\n\torthogonal: %d\n\tleftHanded: %d\n\tstyle: %d\n\tuvName: %s",
			polyframe_parms.which, polyframe_parms.names[2], polyframe_parms.names[1], polyframe_parms.names[2], orthogonal, left_handed, style, polyframe_parms.uv_name);
	}

	/// import cvex
	if (cvexnum > CVEX_MAX_NUM)
	{
		VRAYerror("Wrong CVEX number.");
		return 0;
	}

	std::string cvex_basename = "CVEX";
	std::string runtype_basename = "CVEX_type";
	for (int i = 1; i <= cvexnum; ++i)
	{
		UT_StringHolder cvexfile;
		int runtype;
		std::string cvex_name = cvex_basename + std::to_string(i);
		std::string runtype_name = runtype_basename + std::to_string(i);

		// import and store augments
		if (import(cvex_name.c_str(), cvexfile) && import(runtype_name.c_str(), &runtype, 1))
		{
			cvexfiles.push_back(cvexfile);
			cvex_runtypes.push_back(runtype);
			VRAYprintf(0, "Import CVEX %s as run type %d.", cvexfile.c_str(), runtype);
		}
	}

	///  create child procedurals
	childDeformer_list.clear();
	if (!is_pCloud)
	{
		CVEXExtraAttribMap cvex_extraAttribMap;	// create extraAttribMap but do nothing
		for (int instanceid = 0; instanceid < instancenum; ++instanceid)
		{
			childDeformer_list.push_back(new RAY_Deform(UT_Vector3(0, 0, 0), inputfile, 
				cvexfiles, cvex_extraAttribMap, 
				cvex_runtypes, cvexnum, isMultiThreads, instanceid,
				is_compute_normal, is_velBlur, geo_timeSample, camshutter[0], camshutter[1], fps, 
				polyframe_flags, polyframe_parms));
		}
	}
	else
	{
		/// test
		std::vector<UT_Vector3> positions;
		std::vector<UT_StringHolder> instancefiles;
		// load point cloud
		if (!loadPointCloud(inputfile, positions, instancefiles, attribmaps)) { return 0; }
		assert((positions.size() == instancefiles.size() && positions.size() == attribmaps.size()) 
			&& "Point cloud positions and instancefiles don't match.");
		
		// create instance geo based on point cloud
		for (int instanceid = 0; instanceid < positions.size(); ++instanceid)
		{
			inputfile = instancefiles[instanceid];
			UT_Vector3 pos = positions[instanceid];
			CVEXExtraAttribMap attribmap = *(attribmaps[instanceid]);
			childDeformer_list.push_back(new RAY_Deform(pos, inputfile, 
				cvexfiles, attribmap,
				cvex_runtypes, cvexnum, isMultiThreads, instanceid,
				is_compute_normal, is_velBlur, geo_timeSample, camshutter[0], camshutter[1], fps, 
				polyframe_flags, polyframe_parms));
		}
	}

	return 1;
}

void RAY_DeformInstance::getBoundingBox(UT_BoundingBox &box)
{
	for (auto childProc : childDeformer_list)
	{
		childProc->getBoundingBox(bbox);
	}
	box = bbox;
}

void RAY_DeformInstance::render()
{
	int instanceid = 0;
	for (auto childProc : childDeformer_list)
	{
		// create a new procedural object
		VRAY_ProceduralChildPtr obj = createChild();
		// RAYprintf(0, "Run child procedural instance id: %d ...", instanceid);
		obj->addProcedural(childProc);
		// RAYprintf(0, "Run child procedural instance id: %d complete.", instanceid);
		instanceid++;
	}
}

bool RAY_DeformInstance::loadPointCloud(UT_StringHolder& filename, 
	std::vector<UT_Vector3>& positions, 
	std::vector<UT_StringHolder>& instancefiles, 
	std::vector<CVEXExtraAttribMap*>& attribmaps)
{
	GU_Detail* gd = new GU_Detail();

	// Load geometry from disk
	if (!gd->load(filename, 0).success())
	{
		VRAYerror("Unable to load point cloud: %s", filename.c_str());
		return false;
	}

	int pointNum = gd->getNumPoints();
	VRAYprintf(0, "Create %d instances based on point cloud.", pointNum);
	positions.resize(pointNum);
	instancefiles.resize(pointNum);
	attribmaps.resize(pointNum);

	// pick pos and instancefile
	if (!getTypedPointAttrib(gd, "P", positions, pointNum))
	{
		VRAYwarning("No position information in point cloud.");
		return false;
	}
	if (!getStringPointAttrib(gd, PC_INSTANCEFILE_ATTRIB, instancefiles, pointNum))
	{
		VRAYwarning("No instancefile information in point cloud.");
		return false;
	}

	// set attrib map
	std::vector<GA_Attribute*> geoattriblist;
	for (GA_AttributeDict::iterator it = gd->pointAttribs().begin(); !it.atEnd(); ++it)
	{
		geoattriblist.push_back(it.attrib());
	}
	for (int pointid = 0; pointid < pointNum; ++pointid)
	{
		CVEXExtraAttribMap* attribmap = new CVEXExtraAttribMap;
		attribmaps[pointid] = attribmap;
		for (auto attrib : geoattriblist)
		{
			UT_StringHolder binded_attrib_name = std::string(PC_ATTRIB_PREFIX) + attrib->getName().c_str();
			if (attrib->getStorageClass() == GA_STORECLASS_FLOAT && attrib->getTupleSize() < 3)
			{
				GA_ROHandleT<fpreal32> handle(gd, GA_ATTRIB_POINT, attrib->getName());
				GA_Offset off = gd->pointOffset(GA_Index(pointid));
				if (handle.isValid())
				{
					attribmap->floatAttribMap[binded_attrib_name] = handle.get(off);
				}
			}
			else if (attrib->getStorageClass() == GA_STORECLASS_FLOAT && attrib->getTupleSize() < 4)
			{
				GA_ROHandleT<UT_Vector3> handle(gd, GA_ATTRIB_POINT, attrib->getName());
				GA_Offset off = gd->pointOffset(GA_Index(pointid));
				if (handle.isValid())
				{
					attribmap->vec3AttribMap[binded_attrib_name] = handle.get(off);
				}
			}
			else if (attrib->getStorageClass() == GA_STORECLASS_FLOAT)
			{
				GA_ROHandleT<UT_Vector4> handle(gd, GA_ATTRIB_POINT, attrib->getName());
				GA_Offset off = gd->pointOffset(GA_Index(pointid));
				if (handle.isValid())
				{
					attribmap->vec4AttribMap[binded_attrib_name] = handle.get(off);
				}
			}
			else if (attrib->getStorageClass() == GA_STORECLASS_INT)
			{
				GA_ROHandleT<int> handle(gd, GA_ATTRIB_POINT, attrib->getName());
				GA_Offset off = gd->pointOffset(GA_Index(pointid));
				if (handle.isValid())
				{
					attribmap->intAttribMap[binded_attrib_name] = handle.get(off);
				}
			}
		}
	}

	delete gd;

	return true;
}

bool RAY_DeformInstance::getStringPointAttrib(GU_Detail *gd, UT_StringHolder name, std::vector<UT_StringHolder>& inputlist, int size)
{
	GA_ROHandleS handle(gd, GA_ATTRIB_POINT, name);
	GA_Offset off = gd->pointOffset(GA_Index(0));
	if (handle.isValid())
	{
		const char* attr;
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
		VRAYwarningOnce("Handle for attribute %s is not valid.", name.c_str());
		return false;
	}
}
