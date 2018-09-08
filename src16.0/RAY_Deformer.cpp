
#include "RAY_Deformer.h"

using namespace HDK_Deform;


//** child procedural: deformer for single instance

RAY_Deform::RAY_Deform(UT_Vector3 center, UT_StringHolder infile,
	std::vector<UT_StringHolder>& cfiles, CVEXExtraAttribMap& cextra,
	std::vector<int>& cruntypes, int cvexn, int isMultiT, int ins,
	int compN, int isVB, int geoTSample, fpreal open, fpreal close, fpreal fps,
	int* polyframeflags, const GU_PolyFrameParms& pf_parms):

	isSuccess(true), 
	center_pos(center), 
	inputfile(infile), 
	cvexfiles(cfiles), 
	cvex_extraAttribs(cextra), 
	cvex_runtypes(cruntypes), 
	instance_id(ins), 
	cvex_num(cvexn), 
	is_multi_threads(isMultiT), 
	is_compute_normal(compN), 
	is_velBlur(isVB), 
	geo_timeSample(geoTSample), 
	camShutter_open(open),
	camShutter_close(close), 
	fps(fps), 
	polyframe_flags(polyframeflags),
	polyframe_parms(pf_parms)
{
	bbox.initBounds(0.0, 0.0, 0.0);
	if (preprocess() == 0) { isSuccess = false; }	// calculate bbox during child construction
}

RAY_Deform::~RAY_Deform()
{
	cleanBuffer();
}

const char* RAY_Deform::className() const
{
	return "RAY_Deform";
}

// child procedural: this function never be called
int RAY_Deform::initialize(const UT_BoundingBox *box)
{
	return 0;
}

void RAY_Deform::getBoundingBox(UT_BoundingBox &box)
{
	box.enlargeBounds(bbox);
	// box = bbox;
}

void RAY_Deform::render()
{
	if (!isSuccess) { return; }

	/// motion blur
	// velocity motion blur: can only run in render() somehow...
	if (is_velBlur)
	{
		/// motion blur
		fpreal preBlur = -(camShutter_open) / fps;
		fpreal postBlur = (camShutter_close) / fps;
		geo.addVelocityBlur(preBlur, postBlur);
	}

	VRAY_ProceduralChildPtr obj = createChild();
	obj->addGeometry(geo);
}

/// preprocess

int RAY_Deform::preprocess()
{
	/// load geo from file
	if (!loadGeo()) { return 0; }
	GU_Detail* gd = geo.get();

	std::vector<GU_Detail*> gdlist;
	std::vector<fpreal32> shutterlist;
	gdlist.push_back(gd);
	shutterlist.push_back((fpreal32)camShutter_open);		// default shutter time for no motion blur is 0.0

	/// motion blur
	if (!is_velBlur)
	{
		fpreal shutter_time = camShutter_close - camShutter_open;
		if (geo_timeSample > 0 && shutter_time > 0.0)
		{
			fpreal shutter_step;
			if (geo_timeSample <= 1) { shutter_step = 0; }
			else
			{
				shutter_step = shutter_time / (geo_timeSample - 1);
			}
			for (int i = 1; i < geo_timeSample; ++i)
			{
				fpreal curr_shutter_norm = (i * shutter_step) / shutter_time;

				auto g1 = geo.appendSegmentGeometry(curr_shutter_norm);	// pass normalized shutter (scale from 0 to 1)
				GU_DetailHandleAutoWriteLock wlock(g1);
				if (!wlock.getGdp()->load(inputfile, 0).success())
				{
					VRAYerror("Unable to load motion blur segment geometry[%d]: %s", i, inputfile.c_str());
					geo.removeSegmentGeometry(g1);
				}
				else
				{
					shutterlist.push_back((fpreal32)(camShutter_open + i * shutter_step));
					gdlist.push_back(wlock.getGdp());
				}
			}
		}
	}

    // fpreal32 current_shutter;
	/// execute cvex on different shutter GU_Details
	for (int guid = 0; guid < gdlist.size(); ++guid)
	{
		GU_Detail* current_gd = gdlist[guid];
        fpreal32 *current_shutter = shutterlist.data() + guid;

		// move to center
		current_gd->translate(center_pos);

		/// pre polyframe
		if (polyframe_flags[0])	{ /*for (auto current_gd : gdlist)*/	{ polyFrame(current_gd);} }

		/// execute different cvex files
		for (int i = 0; i < cvexfiles.size(); ++i)
		{
			// RAYprintf(0, "=== Start execute cvex id: %d ===", i);
			inputcvex = cvexfiles[i];
			cvex_runtype = cvex_runtypes[i];

			// execute cvex
			if (cvex_runtype == DO_POINTS || cvex_runtype == DO_PRIMS || cvex_runtype == DO_VERTS || cvex_runtype == DO_DETAILS) 
				{ executeCVEX(current_gd, current_shutter); }
			else { VRAYprintf(0, "No valid cvex. Create geometry instance only."); }

			// clean buffer for cvex
			cleanBuffer();

			// post polyframe after each cvex
			if (polyframe_flags[i + 1]) { /*for (auto current_gd : gdlist)*/ { polyFrame(current_gd); } }
		}
	}

	/// update bbox
	gd->getBBox(&bbox);
	if (is_velBlur)
	{
		// update bbox based on "v" attribute
		velBBox(gd, bbox);
	}
	else
	{
		// add bbox from segment geo
		for (int guid = 1; guid < gdlist.size(); ++guid)
		{
			UT_BoundingBox segmentBBox;
			GU_Detail* current_gd = gdlist[guid];
			current_gd->getBBox(&segmentBBox);
			bbox.enlargeBounds(segmentBBox);
		}
	}

	/// re-compute normal
	if (is_compute_normal)
	{
		// gd->normal();
		for (auto current_gd : gdlist)
		{
			current_gd->normal();
		}
	}

	return 1;
}

void RAY_Deform::cleanBuffer()
{
	// delete inputs
	for (auto buffer : inputbuffer)
	{
		for (auto pt : buffer)
		{
			delete[] pt;
		}
		buffer.clear();
	}
	inputbuffer.clear();
	// delete outputs
	for (auto buffer : vec3_outputbuffer)
	{
		for (const auto& attribinfo : buffer)
		{
			delete[] attribinfo.second;
		}
		buffer.clear();
	}
	for (auto buffer : float_outputbuffer)
	{
		for (const auto& attribinfo : buffer)
		{
			delete[] attribinfo.second;
		}
		buffer.clear();
	}
	for (auto buffer : vec4_outputbuffer)
	{
		for (const auto& attribinfo : buffer)
		{
			delete[] attribinfo.second;
		}
		buffer.clear();
	}
	for (auto buffer : int_outputbuffer)
	{
		for (const auto& attribinfo : buffer)
		{
			delete[] attribinfo.second;
		}
		buffer.clear();
	}
	vec3_outputbuffer.clear();
	float_outputbuffer.clear();
	vec4_outputbuffer.clear();
	int_outputbuffer.clear();
	// delete attribute holder
	geoattriblist.clear();
	cvexoutputnamelist.clear();
}

/// Geo

bool RAY_Deform::loadGeo()
{
	geo = createGeometry();
	
	// Load geometry from disk
	if (!geo->load(inputfile, 0).success())
	{
		VRAYerror("Unable to load geometry[0]: %s", inputfile.c_str());
		return false;
	}
	// RAYprintf(0, "Load geometry[0] success: %s", inputfile.c_str());
	
	return true;
}

/// bbox

void RAY_Deform::velBBox(GU_Detail *gd, UT_BoundingBox& box)
{
	UT_BoundingBox testbox;
	// traverse the geo and cal pos displacement based on "v" attribute
	int size = gd->getNumPoints();
	GA_AttributeOwner owner = GA_ATTRIB_POINT;
	GA_Offset offset = gd->pointOffset(GA_Index(0));
	UT_Vector3* vel_list = new UT_Vector3[size];
	UT_Vector3* pos_list = new UT_Vector3[size];
	if (getTypedAttribByGeom<UT_Vector3>(gd, "P", pos_list, owner, offset, size) &&
		getTypedAttribByGeom<UT_Vector3>(gd, "v", vel_list, owner, offset, size))
	{
		fpreal preBlur = -(camShutter_open) / fps;
		fpreal postBlur = (camShutter_close) / fps;
		for (int i = 0; i < size; ++i)
		{
			UT_Vector3 prePos = pos_list[i] - vel_list[i] * preBlur;
			UT_Vector3 postPos = pos_list[i] + vel_list[i] * postBlur;
			box.enlargeBounds(prePos);
			box.enlargeBounds(postPos);
		}
	}
	delete[] vel_list;
	delete[] pos_list;
}

/// PolyFrame

void RAY_Deform::polyFrame(GU_Detail *gd)
{
	GU_PolyFrame polyframe(gd);
    GU_PolyFrameError err = polyframe.computeFrames(polyframe_parms);
	switch (err)
	{
	case GU_POLYFRAME_CREATE_ATTRIBUTE_FAILED:
		VRAYerror("PolyFrame create attribute failed.");
		break;
	case GU_POLYFRAME_MISSING_DETAIL:
		VRAYerror("PolyFrame missing detail.");
		break;
	case GU_POLYFRAME_MISSING_TEXTURE_COORDS:
		VRAYerror("PolyFrame missing texture coords.");
		break;
	default:
		break;
	}
}


/// CVEX

void RAY_Deform::executeCVEX(GU_Detail *gd, fpreal32* shutter)
{
	int total_size;
	// set cvex size
	// cvextype: 0-points, 1-primitives, 2-vertices
	switch (cvex_runtype)
	{
	case DO_POINTS:
		total_size = gd->getNumPoints();
		break;
	case DO_PRIMS:
		total_size = gd->getNumPrimitives();
		break;
	case DO_VERTS:
		total_size = gd->getNumVertices();
		break;
	case DO_DETAILS:
		total_size = 1;
		break;
	default:
		break;
	}
	// RAYprintf(0, "CVEX size: %d", total_size);

	int thread_num = total_size / CHUCK_SIZE;
	if (total_size % CHUCK_SIZE != 0) { thread_num++; }

	// set buffer for each threads
	inputbuffer.resize(thread_num);
	vec3_outputbuffer.resize(thread_num);
	float_outputbuffer.resize(thread_num);
	vec4_outputbuffer.resize(thread_num);
	int_outputbuffer.resize(thread_num);

	/// single thread
	if (!is_multi_threads)
	{
		// init
		CVEX_Context cvex;
		CVEX_RunData rundata;
		UT_Array<exint> procid(total_size, total_size);
		rundata.setProcId(procid.array());
		VEX_GeoCommandQueue geocmd;
		geocmd.myNumPrim = gd->getNumPrimitives();
		geocmd.myNumVertex = gd->getNumVertices();
		geocmd.myNumPoint = gd->getNumPoints();
		rundata.setGeoCommandQueue(&geocmd);
		processCVEX(cvex, rundata, gd, 0, total_size, 0, shutter);
		// gvex
		GVEX_GeoCommand allcmd;
		allcmd.appendQueue(geocmd);
		allcmd.apply(gd);

		return;
	}

	/// multi threads
	// parse geom attributes
	getGeomAttribs(gd);	// only run before multi-threads
	CVEX_Context cvex;
	// add cvex input
	addCVEXInput(cvex);
	// load cvex
	if (!loadCVEX(cvex)) { return; }
	// create new outputs based on cvex function and create corresponding attrib for geom
	createGeomAttribFromCVEXOutput(cvex, gd);	// only run before multi-threads

	VEX_GeoCommandQueue** threadcmds;
	threadcmds = new VEX_GeoCommandQueue*[thread_num];
	CVEXProcessData** procDataBuffer = new CVEXProcessData*[thread_num];

	UT_PtrArray<UT_Thread*> threads;
	void* data = nullptr;
	for (int tid = 0; tid < thread_num; tid++)
	{
		procDataBuffer[tid] = new CVEXProcessData(this, gd, threadcmds, tid, total_size, thread_num, *shutter);
		data = procDataBuffer[tid];
		threads.append(UT_Thread::allocThread(UT_Thread::ThreadLowUsage));
		if (!threads(tid) || !threads(tid)->startThread(&executeSingleCVEX, data))
		{
			VRAYerror("Failed to create thread %d", tid);
			break;
		}
	}
	// Wait for all threads to finish, deallocate them once done
	for (int i = 0; i < threads.entries(); i++)
	{
		threads(i)->waitForState(UT_Thread::ThreadIdle);
		delete threads(i);
	}
	// gvex
	GVEX_GeoCommand allcmd;
	for (int tid = 0; tid < thread_num; ++tid)
	{
		allcmd.appendQueue(*threadcmds[tid]);
	}
	allcmd.apply(gd);

	// clean memory
	for (int tid = 0; tid < thread_num; ++tid)
	{
		delete threadcmds[tid];
		delete procDataBuffer[tid];
	}
	delete[] threadcmds;
	delete[] procDataBuffer;
}

bool RAY_Deform::processCVEX(CVEX_Context &context, CVEX_RunData &rundata, GU_Detail *gd, int gid, int size, int tid, fpreal32* shutter)
{
	// parse geom attrib
	getGeomAttribs(gd);
	// add cvex input
	addCVEXInput(context);
	// load cvex
	if (!loadCVEX(context)) { return false; }
	// create new outputs based on cvex function and create corresponding attrib for geom
	createGeomAttribFromCVEXOutput(context, gd);
	// allocate memory for input and output
	findCVEX(context, gd, gid, size, tid, shutter);

	// run cvex program
	context.run(size, true, &rundata);
	// pass cvex result back to geom
	setCVEXOutput(context, gd, gid, size, tid);

	return true;
}

void RAY_Deform::getGeomAttribs(GU_Detail *gd)
{
	/// get geom attributes
	// cvextype: 0-points, 1-primitives, 2-vertices
	switch (cvex_runtype)
	{
		// points
	case DO_POINTS:
	{
		for (GA_AttributeDict::iterator it = gd->pointAttribs().begin(); !it.atEnd(); ++it)
		{
			geoattriblist.push_back(it.attrib());
		}
		break;
	}
	// prims
	case DO_PRIMS:
	{
		for (GA_AttributeDict::iterator it = gd->primitiveAttribs().begin(); !it.atEnd(); ++it)
		{
			geoattriblist.push_back(it.attrib());
		}
		break;
	}
	// verts
	case DO_VERTS:
	{
		for (GA_AttributeDict::iterator it = gd->vertexAttribs().begin(); !it.atEnd(); ++it)
		{
			geoattriblist.push_back(it.attrib());
		}
		break;
	}
	case DO_DETAILS:
	{
		for (GA_AttributeDict::iterator it = gd->attribs().begin(); !it.atEnd(); ++it)
		{
			geoattriblist.push_back(it.attrib());
		}
		break;
	}
	default:
		break;
	}
}

void RAY_Deform::addCVEXInput(CVEX_Context &context)
{
	/// add instance and shutter as uniform input
	context.addInput("instance", CVEX_TYPE_INTEGER, false);
	context.addInput("shutter", CVEX_TYPE_FLOAT, false);

	/// add extra uniform input
	for (const auto & attribinfo : cvex_extraAttribs.floatAttribMap)
	{
		context.addInput(attribinfo.first, CVEX_TYPE_FLOAT, false);
	}
	for (const auto & attribinfo : cvex_extraAttribs.intAttribMap)
	{
		context.addInput(attribinfo.first, CVEX_TYPE_INTEGER, false);
	}
	for (const auto & attribinfo : cvex_extraAttribs.vec3AttribMap)
	{
		context.addInput(attribinfo.first, CVEX_TYPE_VECTOR3, false);
	}
	for (const auto & attribinfo : cvex_extraAttribs.vec4AttribMap)
	{
		context.addInput(attribinfo.first, CVEX_TYPE_VECTOR4, false);
	}
	/// add cvex inputs from geom attributes
	for (auto attrib : geoattriblist)
	{
		CVEX_Type type = attrib2CVEXTypeHandler(attrib);
		if (type != CVEX_TYPE_INVALID)
		{
			context.addInput(attrib->getName(), attrib2CVEXTypeHandler(attrib), true);
		}
	}
}

bool RAY_Deform::loadCVEX(CVEX_Context &context)
{
	// pass shoppath
	UT_String shoppath = UT_String(inputcvex);

	/// load cvex
	char* argv[4096];
	int argc = shoppath.parse(argv, 4096);
	if (!context.load(argc, argv))	// Pass arguments to CVEX
	{
		VRAYerrorOnce("CVEX %s as runtype %d: Cannot load CVEX.", inputcvex.c_str(), cvex_runtype);
		return false;
	}
	// VRAYprintf(0, "Load cvex success: %s", shoppath.c_str());
	return true;
}

void RAY_Deform::findCVEX(CVEX_Context &context, GU_Detail *gd, int gid, int size, int tid, fpreal32* shutter)
{
	/// set instance input
	findTypedUniformInput(context, "instance", &instance_id);
	findTypedUniformInput(context, "shutter", shutter);

	/// set extra uniform input
	for (const auto & attribinfo : cvex_extraAttribs.floatAttribMap)
	{
		findTypedUniformInput(context, attribinfo.first, (fpreal32*)&(attribinfo.second));
	}
	for (const auto & attribinfo : cvex_extraAttribs.intAttribMap)
	{
		findTypedUniformInput(context, attribinfo.first, (int*)&(attribinfo.second));
	}
	for (const auto & attribinfo : cvex_extraAttribs.vec3AttribMap)
	{
		findTypedUniformInput(context, attribinfo.first, (UT_Vector3*)&(attribinfo.second));
	}
	for (const auto & attribinfo : cvex_extraAttribs.vec4AttribMap)
	{
		findTypedUniformInput(context, attribinfo.first, (UT_Vector4*)&(attribinfo.second));
	}
	/// set geom attrib inputs and outputs
	findTypedCVEX(context, gd, vec3_outputbuffer, gid, size, tid);
	findTypedCVEX(context, gd, float_outputbuffer, gid, size, tid);
	findTypedCVEX(context, gd, vec4_outputbuffer, gid, size, tid);
	findTypedCVEX(context, gd, int_outputbuffer, gid, size, tid);
}

void RAY_Deform::setCVEXOutput(CVEX_Context &context, GU_Detail *gd, int gid, int size, int tid)
{
	/// set geom attrib outputs back to geom
	setTypedCVEXOutput(context, gd, vec3_outputbuffer, gid, size, tid);
	setTypedCVEXOutput(context, gd, float_outputbuffer, gid, size, tid);
	setTypedCVEXOutput(context, gd, vec4_outputbuffer, gid, size, tid);
	setTypedCVEXOutput(context, gd, int_outputbuffer, gid, size, tid);
}

CVEX_Type RAY_Deform::attrib2CVEXTypeHandler(GA_Attribute* attrib)
{
	if (attrib->getStorageClass() == GA_STORECLASS_FLOAT && attrib->getTupleSize() < 3) { return CVEX_TYPE_FLOAT; }
	if (attrib->getStorageClass() == GA_STORECLASS_FLOAT && attrib->getTupleSize() < 4) { return CVEX_TYPE_VECTOR3; }
	if (attrib->getStorageClass() == GA_STORECLASS_FLOAT) { return CVEX_TYPE_VECTOR4; }
	if (attrib->getStorageClass() == GA_STORECLASS_INT) { return CVEX_TYPE_INTEGER; }
	return CVEX_TYPE_INVALID;
}

void RAY_Deform::createGeomAttribFromCVEXOutput(CVEX_Context &context, GU_Detail *gd)
{
	GA_AttributeOwner owner;
	switch (cvex_runtype)
	{
	case DO_POINTS:
	{
		owner = GA_ATTRIB_POINT;
		break;
	}
	case DO_PRIMS:
	{
		owner = GA_ATTRIB_PRIMITIVE;
		break;
	}
	case DO_VERTS:
	{
		owner = GA_ATTRIB_VERTEX;
		break;
	}
	case DO_DETAILS:
	{
		owner = GA_ATTRIB_DETAIL;
		break;
	}
	default:
		VRAYerrorOnce("CVEX %s as runtype %d: Cannot identify CVEX run type.", inputcvex.c_str(), cvex_runtype);
	}

	CVEX_ValueList& value_list = context.getOutputList();
	CVEX_Value  *value;

	for (int i = 0; i < value_list.entries(); ++i)
	{
		value = value_list.getValue(i);
		// skip output value which is not exported
		if (!value->isExport())	continue;
		// create attribute for geom
		UT_StringHolder name = value->getName();
		// auto it = std::find_if(geoattriblist.begin(), geoattriblist.end(), [name](const auto& val) {return val->getName() == name; });   // C++14
		auto it = std::find_if(geoattriblist.begin(), geoattriblist.end(), [name](const GA_Attribute* val) {return val->getName() == name; });
		// not find the attribute
		if (!(it != geoattriblist.end()))
		{
			int attrib_length = 1;
			switch (value->getType())
			{
				case CVEX_TYPE_VECTOR3:
				{
					attrib_length = 3;
					break;
				}
				case CVEX_TYPE_VECTOR4:
				{
					attrib_length = 4;
					break;
				}
				default:	// float and int: length = 1
				{
					attrib_length = 1;
					break;
				}
			}
			gd->addFloatTuple(owner, GA_SCOPE_PUBLIC, value->getName(), attrib_length);
		}
		// set cvex output list
		cvexoutputnamelist.push_back(name);
	}
}
