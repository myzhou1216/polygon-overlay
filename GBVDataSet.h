// GBVDataSet.h: interface for the CGBVDataSet class.
//
//////////////////////////////////////////////////////////////////////


#if !defined(AFX_GBVDATASET_H__37037DA9_D826_4C26_ACF6_692FAD69ADF9__INCLUDED_)
#define AFX_GBVDATASET_H__37037DA9_D826_4C26_ACF6_692FAD69ADF9__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "ogrsf_frmts.h"
#include <math.h>
#include "VWDrawGeoLayer.h"
#include "VWAngle.h"
#include "RealVectorTile.h"

class __declspec(dllexport)CBoundBox
{
public:
	CBoundBox();
	CBoundBox(double xmax, double xmin, double ymax, double ymin);
	bool Intersect(CBoundBox other); //ÓÐ½»¼¯

	double max_x,min_x,max_y,min_y;
};


typedef struct tagVGeometry
{
	unsigned long ID;
	CBoundBox m_box;
	std::vector<double> m_pointAry;
	CString Name;
}VGeometry;

typedef std::vector<int> NodeIndex;

struct RVPolygon
{
	unsigned long ID;
	std::vector<double> vertices;
	CString Name;
};

struct RVRenderData
{
	unsigned long ID;
	std::vector<RVPoint> pointAry;
	CString Name;
};

enum RVGeometryType
{
	RV_Point = 0,
		RV_Line = 1,
		RV_Polygon = 2,
		RV_UnKnown = 3
};


class CVWDrawGeoLayer;

class __declspec(dllexport) CGBRVDataSet: public CVWSceneNode
{
public:
	CGBRVDataSet();
	virtual ~CGBRVDataSet();

	BOOL m_bShp;
	double m_nHalfCountLati;
	double                        m_dMinVisibleLongitude;
	double                        m_dMaxVisibleLongitude; 
	double                        m_dMinVisibleLatitude;
	double                        m_dMaxVisibleLatitude;
	double                        m_dLongitudeInterval;
	double                        m_dLatitudeInterval;
	CVWSceneViewer		*m_pSceneViewer;
	CVWTerrainAccessor *m_pTerrainAccessor;
	double	m_dEquatorialRadius;
	float **m_ppHeightData;
	double m_fDistanceAboveSurface;

public:
	bool OpenVectorFile(const char *filename);
	double DisPointToLine(double x, double y, double x1, double y1, double x2, double y2);
	void DouglasLine(float tolerance, std::vector<double> &points, std::vector<bool> &bSel, int startpos, int endpos);
	bool DouglasPolygon(float tolerance, const std::vector<double> &points, std::vector<double> &output);
	void Simplify2D(float tolerance);
	
public:
	void GetPointsFromRect(std::vector<RVRenderData> &ptAry, double minLat, double minLong, double maxLat, double maxLong);
	void GetLinesFromRect(std::vector<RVRenderData> &ldtAry, std::vector<NodeIndex> &indexAry, double minLat, double minLong, double maxLat, double maxLong);
	void GetLinesFromRect(std::vector<RVRenderData> &ldtAry, double minLat, double minLong, double maxLat, double maxLong);
	void GetPolygonsFromRect(std::vector<RVRenderData> &ldtAry, std::vector<RVPolygon> &plgAry, double minLat, double minLong, double maxLat, double maxLong);
	void GetPolygonsFromRect(std::vector<RVRenderData> &ldtAry, double minLat, double minLong, double maxLat, double maxLong);

	//weiler-atherton
	void GetPolygonsFromRect2(std::vector<RVRenderData> &ldtAry, std::vector<RVPolygon> &plgAry, double minLat, double minLong, double maxLat, double maxLong);
	void GetPolygonsFromRect2(std::vector<RVRenderData> &ldtAry, double minLat, double minLong, double maxLat, double maxLong);
	
	//Cohen-Sutherland
	static void Code(double x,double y,int *c,double minx, double miny, double maxx, double maxy);
	//Cohen-Sutherland
	static int LineIntersectRect(double p1x,double p1y,double p2x,double p2y,double minx,
		double miny,double maxx,double maxy,double *x1,double *y1,double *x2,double *y2);

 	RVPoint* GetNewTerrainData(double minLat, double minLong, double maxLat, double maxLong);
	void LineInterpolating(double x1,double y1,double x2,double y2,RVPoint *vp,std::vector<RVPoint> &ptAry);

	void Simplification(double tolerance,double raise,std::vector<RVPoint> &ptAry,std::vector<bool> &bSel);
	void Simplification(double tolerance,double raise,std::vector<RVPoint> &ptAry,std::vector<bool> &bSel,int startpos,int endpos);
	void EliminateCover(double raise,std::vector<RVPoint> &ptAry,std::vector<bool> &bSel,int startpos,int endpos);

	double GetPointHeightInTriangle(double x, double y, RVPoint *vp);

	bool LineRect(double x1, double y1, double x2, double y2, 
		double rtxMin, double rtyMin, double rtxMax, double rtyMax); 
	bool LineIntersectLine(double x1, double y1, double x2, double y2,
		double x3, double y3, double x4, double y4, double *x, double *y);
	bool LineIntersectLine(double x1, double y1, double x2, double y2,
		double x3, double y3, double x4, double y4, double *x, double *y, int *flag);

	inline double Distance(double x1,double y1,double x2,double y2)
	{
		return pow((x1-x2) * (x1-x2) + (y1-y2)*(y1-y2), 0.5);
	}
	inline double Area(double x1,double y1,double x2,double y2,double x3,double y3)
	{
		double area=(x2-x1)*(y3-y1)-(x3-x1)*(y2-y1);
		return 0.5*area; //>0
	}
	inline double fabsArea(double x1,double y1,double x2,double y2,double x3,double y3)
	{
		double area=(x2-x1)*(y3-y1)-(x3-x1)*(y2-y1);
		return fabs(0.5*area); 
	}

	inline CVWSceneViewer *GetCurrentSceneViewer()
	{
		if(m_pSceneViewer != NULL)
		{
			m_pSceneViewer->AddRef();
		}
		return m_pSceneViewer;
	}
	
	inline void SetCurrentSceneViewer(CVWSceneViewer *pSceneViewer)
	{
		if(m_pSceneViewer != NULL)
		{
			m_pSceneViewer->Release();
		}
		m_pSceneViewer = pSceneViewer;
		if(m_pSceneViewer != NULL)
		{
			m_pSceneViewer->AddRef();
		}
}

public:
	bool m_bOpening;
	double	m_dEast,m_dWest,m_dNorth,m_dSouth;
	std::vector<VGeometry> m_Dsfromfile;	
	std::vector<VGeometry> m_DsSimplified;	
	RVGeometryType  m_GeoType;

public:
	bool GetCrossPnt(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double *x, double *y);
	int TriangulateConvex2(RVLine &TriangleVerts,RVLine PolyVert);
	void GetLinesFromTri(RVLine &pntAry, RVLine tempLine, RVPoint startPoint, RVPoint endPoint);
	void GetIntersectionOfTris2(RVLine &pntAry, RVPoint tri1[4], RVPoint tri2[4]);
	double LeftOrRight(double x,double y,double x1,double y1,double x2,double y2);
	bool LineIntersectLineTris(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double *x, double *y,int *flag, bool &bnode);
	bool OutJdOfTris3(RVPoint tri1[4], RVPoint tri2[4],RVLine &intersection);
	double m_dZScale;
	RVLine m_AnalysisResultPnts;
	BOOL GetIntersectionOfFeatures(RVAnalysisPolyData analysisData1, RVAnalysisPolyData analysisData2);
	bool m_bVectorAnalysis;
	void GetIntersectionOfTris(RVLine &pntAry,RVPoint tri1[4],RVPoint tri2[4]);
	void Render(CVWSceneViewer *pSceneViewer);
	BOOL PerformSelectionAction(CVWSceneViewer *pSceneViewer);
	void Update(CVWSceneViewer *pSceneViewer);
	BOOL Initialize(CVWSceneViewer *pSceneViewer);
	CVWTerrainAccessor * GetTerrainAccessor();
	void SetTerrainAccessor(CVWTerrainAccessor *pTerrainAccessor);
	void ComputeGridValues(CVWSceneViewer* pSceneViewer);

	double GetZInArea(double x, double y, RVPoint pnt0, RVPoint pnt1, RVPoint pnt2);
	double GetZOnLine(double x,double y,RVPoint pnt0,RVPoint pnt1);
	bool GetTrianglesFromRect(std::vector<double> p1,RVPoint *grid,std::vector<RVPoint> &pntAry);
	bool PntInPoly(double x, double y, std::vector<RVPointFlag> p3, int nvert);
	bool OutJdOfTris2(RVLineFlag p3,RVLineFlag p4,RVLineFlag &subpoly);
	bool OutJd2(std::vector<RVPointFlag> p3,std::vector<RVPointFlag> p4,std::vector<RVPointFlag> &subpoly);

	int TriangulateConvex(std::vector<RVPoint> &TriangleVerts,RVLineFlag SubPolyVert);
	int Triangulate(std::vector<RVPoint> &TriangleVerts,std::vector<RVPointFlag> SubPolyVert,int nBeginVert,int nEndVert);
	bool PointInTriangle(double x, double y, double x1, double y1, double x2, double y2, double x3, double y3);
	int IsAntiClockwise(double x1, double y1, double x2, double y2, double x3, double y3);
	bool OutJd1(std::vector<RVPointFlag> p3,std::vector<RVPointFlag> p4,std::vector<RVPointFlag> &subpoly);
	void InsertNewLine(std::vector<double> p1, RVPoint *p2, std::vector<RVPointFlag> &p3, std::vector<RVPointFlag> &p4);//20140527
	bool InsertNewLineTris(RVPoint *tri1, RVPoint *tri2, RVLineFlag &p3, RVLineFlag &p4);//20151106


	CString GetFeatureName(OGRFeature* poFeature);
	void GetPolarXYPos(double x, double y,double dLatSpan, double *xPos, double *yPos);
	RVPoint* GetPolarTerrainData(double minLat, double minLong, double maxLat, double maxLong);
	bool m_bLoadTerrain;


	int m_nSubPolyCount;
	int ct;
	int ct3;
	int ct4;
	int ctt[200];
	
	static int m_nxVertexCount;
	static int m_nyVertexCount;
	static int m_nxGridCount;
	static int m_nyGridCount;
	static int m_nVertexCount;
	static int m_nGridCount;
	int m_nRow,m_nCol,m_nLevel;
	int m_nQQMLevel,m_nBeginLevel;
	int m_nPole;
	bool m_bMatch;	
	RVPoint *m_pVpoints;
	char *m_pTileData;

	bool m_bCompress2D;


	double m_dOffsetX;
	double m_dOffsetY;


};

#endif // !defined(AFX_GBVDATASET_H__37037DA9_D826_4C26_ACF6_692FAD69ADF9__INCLUDED_)
