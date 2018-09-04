
#ifndef __RAY_DeformInstance__
#define __RAY_DeformInstance__

#include "RAY_Deformer.h"

namespace HDK_Deform
{
	class RAY_Deform;

	class RAY_DeformInstance : public VRAY_Procedural {
	public:
		RAY_DeformInstance();
		virtual ~RAY_DeformInstance();
		virtual const char *className() const;
		virtual int initialize(const UT_BoundingBox *);
		virtual void getBoundingBox(UT_BoundingBox &box);
		virtual void render();

	private:
		/// geom boundingbox
		UT_BoundingBox bbox;
		/// child procedural list
		std::vector<RAY_Deform*> childDeformer_list;
		/// extra attribute map
		std::vector<CVEXExtraAttribMap*> attribmaps;

		bool loadPointCloud(UT_StringHolder& filename, 
			std::vector<UT_Vector3>& positions, 
			std::vector<UT_StringHolder>& instancefiles, 
			std::vector<CVEXExtraAttribMap*>& attribmaps);

		// get attrib value
		bool getStringPointAttrib(GU_Detail *gd, UT_StringHolder name, std::vector<UT_StringHolder>& inputlist, int size);
		template <typename T>
		inline bool getTypedPointAttrib(GU_Detail *gd, UT_StringHolder name, std::vector<T>& inputlist, int size)
		{
			GA_ROHandleT<T> handle(gd, GA_ATTRIB_POINT, name);
			GA_Offset off = gd->pointOffset(GA_Index(0));
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
				VRAYwarningOnce("Handle for attribute %s is not valid.", name.c_str());
				return false;
			}
		}

	};
}

#endif
