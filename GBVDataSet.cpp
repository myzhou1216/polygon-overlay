// GBVDataSet.cpp: implementation of the CGBVDataSet class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "GBVDataSet.h"
#include <algorithm>
#include "VWDrawGeoLayer.h"
#include "VWMath.h"
#include "VWViewFrustum.h"
#include "VWAngle.h"

#include <string>
using namespace std;


#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

double dMinSpan=40;
/////CBoundBox///////////////////////////////////////////////////////
CBoundBox::CBoundBox()
{
	max_x = min_x = max_y = min_y = 0;
}

CBoundBox::CBoundBox(double xmax, double xmin, double ymax, double ymin)
{
	max_x = xmax;
	min_x = xmin;
	max_y = ymax;
	min_y = ymin;
}

bool CBoundBox::Intersect(CBoundBox other)
{
	if (other.max_x < min_x || other.min_x > max_x ||
		other.max_y < min_y || other.min_y > max_y)
		return false;
	else
		return true;
}

////CGBRVDataSet类//////////////////////////////////////////////////////
int CGBRVDataSet::m_nVertexCount = 33;
int CGBRVDataSet::m_nyVertexCount = 33;
int CGBRVDataSet::m_nxVertexCount = 65;
int CGBRVDataSet::m_nGridCount = 32;
int CGBRVDataSet::m_nyGridCount = 32;
int CGBRVDataSet::m_nxGridCount = 64;

CGBRVDataSet::CGBRVDataSet(): CVWSceneNode(_T("DrawShpLayer"))
{
	m_dZScale=10.0;
	m_bVectorAnalysis=false;
	m_bShp=FALSE;
	m_pSceneViewer=NULL;
	m_fDistanceAboveSurface=12.0;
	m_pTerrainAccessor = NULL;
	m_dEquatorialRadius = 6378137.0f;
	m_bOpening = false;
	m_bMatch = false;
	m_bLoadTerrain = false;
	m_GeoType = RV_UnKnown;
	m_pVpoints = NULL;
	m_pTileData = NULL;

	m_bCompress2D = FALSE;

	m_dOffsetX = 0;
	m_dOffsetY = 0;

	m_nPole=0;

	m_ppHeightData = new float*[m_nyVertexCount+1];
	for(int i=0; i<m_nyVertexCount+1; i++)
		m_ppHeightData[i]=new float[m_nxVertexCount+1];
}

CGBRVDataSet::~CGBRVDataSet()
{
	if(m_pTerrainAccessor != NULL)
	{
		m_pTerrainAccessor->Release();
	}

	for(int i=0; i<m_nyVertexCount+1; i++)
		delete []m_ppHeightData[i];
	delete []m_ppHeightData;
	m_ppHeightData=NULL;

	if(m_pVpoints != NULL)
	{
		delete []m_pVpoints;
		m_pVpoints = NULL;
	}
	if(m_pTileData != NULL)
	{
		delete []m_pTileData;
		m_pTileData = NULL;
	}
}

///////////////////////////Drape 2D polygon onto 3D terrains///////////////////////////////////////////////
void CGBRVDataSet::GetPolygonsFromRect2(std::vector<RVRenderData> &ldtAry,
									   double minLat, double minLong, double maxLat, double maxLong)
{
	if(m_bOpening == FALSE)
		return;

	RVPoint *vp = NULL;

	int i=0,j=0,k=0;
	double alt = 0;
	RVPoint grid[4];

	for (i=0; i<m_Dsfromfile.size(); i++)
	{
		if (m_Dsfromfile[i].m_box.max_x+m_dOffsetX<minLong || m_Dsfromfile[i].m_box.min_x+m_dOffsetX>maxLong ||
			m_Dsfromfile[i].m_box.max_y+m_dOffsetY<minLat  || m_Dsfromfile[i].m_box.min_y+m_dOffsetY>maxLat)
		{
			continue;
		}
		else
		{
			RVRenderData plg;
			plg.Name=m_Dsfromfile[i].Name;
			plg.ID = m_Dsfromfile[i].ID;

			if(vp == NULL && m_bLoadTerrain && maxLat-minLat<=dMinSpan)
			{
				if (m_nPole==0)
					vp = GetNewTerrainData(minLat, minLong, maxLat, maxLong);
				else
					vp = GetPolarTerrainData(minLat, minLong, maxLat, maxLong);
			}

			if (vp==NULL)
			{
				grid[0].X=minLong;
				grid[0].Y=maxLat;
				grid[1].X=maxLong;
				grid[1].Y=maxLat;
				grid[2].X=minLong;
				grid[2].Y=minLat;
				grid[3].X=maxLong;
				grid[3].Y=minLat;
				grid[0].Z=grid[1].Z=grid[2].Z=grid[3].Z=0;
				GetTrianglesFromRect(m_Dsfromfile[i].m_pointAry,grid,plg.pointAry);
			} 
			else
			{
				for (int rr=0; rr<m_nyGridCount; rr++)
				{
					for (int cc=0; cc<m_nxGridCount; cc++)
					{
						grid[0] = vp[rr * m_nxVertexCount + cc ];
						grid[1] = vp[rr * m_nxVertexCount + cc + 1];
						grid[2] = vp[(rr+1) * m_nxVertexCount + cc ];
						grid[3] = vp[(rr+1) * m_nxVertexCount + cc+1];

						if (m_Dsfromfile[i].m_box.max_x+m_dOffsetX<grid[2].X || m_Dsfromfile[i].m_box.min_x+m_dOffsetX>grid[1].X ||
							m_Dsfromfile[i].m_box.max_y+m_dOffsetY<grid[2].Y  || m_Dsfromfile[i].m_box.min_y+m_dOffsetY>grid[1].Y)
						{
							continue;
						}
						else
							GetTrianglesFromRect(m_Dsfromfile[i].m_pointAry,grid,plg.pointAry);

					}
				}
			}

			if(plg.pointAry.size() != 0)
			{
				ldtAry.push_back(plg);
				plg.pointAry.clear();
			}
		}
	}
}

void CGBRVDataSet::GetPolygonsFromRect2(std::vector<RVRenderData> &ldtAry, std::vector<RVPolygon> &plgAry, double minLat, double minLong, double maxLat, double maxLong)
{
	if(m_bOpening == FALSE)
		return;
	
	RVPoint *vp = NULL;

	bool bIn = false;
	int i=0,j=0,k=0;
	double alt = 0;
	RVPoint grid[4];

	for (i=0; i<m_Dsfromfile.size(); i++)
	{
		if (m_Dsfromfile[i].m_box.max_x+m_dOffsetX<minLong || m_Dsfromfile[i].m_box.min_x+m_dOffsetX>maxLong ||
			m_Dsfromfile[i].m_box.max_y+m_dOffsetY<minLat  || m_Dsfromfile[i].m_box.min_y+m_dOffsetY>maxLat)
		{
			continue;
		}
		else
		{
			RVRenderData plg;
			plg.Name=m_Dsfromfile[i].Name;
			plg.ID = m_Dsfromfile[i].ID;



			if(vp == NULL && m_bLoadTerrain && maxLat-minLat<=dMinSpan)
			{
				if (m_nPole==0)
					vp = GetNewTerrainData(minLat, minLong, maxLat, maxLong);
				else
					vp = GetPolarTerrainData(minLat, minLong, maxLat, maxLong);
			}

			if (vp==NULL)
			{
				grid[0].X=minLong;
				grid[0].Y=maxLat;
				grid[1].X=maxLong;
				grid[1].Y=maxLat;
				grid[2].X=minLong;
				grid[2].Y=minLat;
				grid[3].X=maxLong;
				grid[3].Y=minLat;
				grid[0].Z=grid[1].Z=grid[2].Z=grid[3].Z=0;
				bIn=GetTrianglesFromRect(m_Dsfromfile[i].m_pointAry,grid,plg.pointAry);
			} 
			else
			{
				for (int rr=0; rr<m_nyGridCount; rr++)
				{
					for (int cc=0; cc<m_nxGridCount; cc++)
					{
						grid[0] = vp[rr * m_nxVertexCount + cc ];
						grid[1] = vp[rr * m_nxVertexCount + cc + 1];
						grid[2] = vp[(rr+1) * m_nxVertexCount + cc ];
						grid[3] = vp[(rr+1) * m_nxVertexCount + cc+1];

						if (m_Dsfromfile[i].m_box.max_x+m_dOffsetX<grid[2].X || m_Dsfromfile[i].m_box.min_x+m_dOffsetX>grid[1].X ||
							m_Dsfromfile[i].m_box.max_y+m_dOffsetY<grid[2].Y  || m_Dsfromfile[i].m_box.min_y+m_dOffsetY>grid[1].Y)
						{
							continue;
						}
						else
							bIn=GetTrianglesFromRect(m_Dsfromfile[i].m_pointAry,grid,plg.pointAry);
					}//cc
				}//rr
			}

			if(plg.pointAry.size() != 0)
			{
				ldtAry.push_back(plg);
				plg.pointAry.clear();
			}

			if(bIn)
			{
				RVPolygon pl;
				pl.Name=m_Dsfromfile[i].Name;
				pl.ID = m_Dsfromfile[i].ID;
				for(int m=0; m<m_Dsfromfile[i].m_pointAry.size()-2; m+=2)
				{
					pl.vertices.push_back(m_Dsfromfile[i].m_pointAry[m]);
					pl.vertices.push_back(m_Dsfromfile[i].m_pointAry[m+1]);

				}			
				plgAry.push_back(pl);
			}
			bIn = false;
		}
	}			
}


////////////////////////////////////compute intersection of polygons/////////////////////////////////
BOOL CGBRVDataSet::GetIntersectionOfFeatures(RVAnalysisPolyData analysisData1, RVAnalysisPolyData analysisData2)
{
	int i,j,k;
	int offset=0;
	
	m_AnalysisResultPnts.clear();
	
	for (i=0;i<analysisData1.TriMetaAry.size();i++)
	{
		int nCount=0;
		__int64 key=analysisData1.TriMetaAry[i].triKey;
		for (j=0;j<analysisData2.TriMetaAry.size();j++)
		{
			if (analysisData2.TriMetaAry[j].triKey==key)
			{
				nCount++;
				if (analysisData1.TriMetaAry[i].bTerTri)
				{
					m_AnalysisResultPnts.push_back(analysisData2.PntAry[j*3]);
					m_AnalysisResultPnts.push_back(analysisData2.PntAry[j*3+1]);
					m_AnalysisResultPnts.push_back(analysisData2.PntAry[j*3+2]);
				}
				else if (analysisData2.TriMetaAry[j].bTerTri)
				{
					m_AnalysisResultPnts.push_back(analysisData1.PntAry[i*3]);
					m_AnalysisResultPnts.push_back(analysisData1.PntAry[i*3+1]);
					m_AnalysisResultPnts.push_back(analysisData1.PntAry[i*3+2]);
				}
				
				else
				{
					RVPoint tri1[4],tri2[4];
					
					tri1[0]=analysisData1.PntAry[i*3];
					tri1[1]=analysisData1.PntAry[i*3+1];
					tri1[2]=analysisData1.PntAry[i*3+2];
					tri1[3]=analysisData1.PntAry[i*3];
					
					tri2[0]=analysisData2.PntAry[j*3];
					tri2[1]=analysisData2.PntAry[j*3+1];
					tri2[2]=analysisData2.PntAry[j*3+2];
					tri2[3]=analysisData2.PntAry[j*3];
					
					double dMinX1=tri1[0].X,dMaxX1=tri1[0].X,dMinY1=tri1[0].Y,dMaxY1=tri1[0].Y;
					double dMinX2=tri2[0].X,dMaxX2=tri2[0].X,dMinY2=tri2[0].Y,dMaxY2=tri2[0].Y;
					for (k=1;k<3;k++)
					{
						dMinX1=min(dMinX1,tri1[k].X);
						dMaxX1=max(dMaxX1,tri1[k].X);
						dMinY1=min(dMinY1,tri1[k].Y);
						dMaxY1=max(dMaxY1,tri1[k].Y);
						
						dMinX2=min(dMinX2,tri2[k].X);
						dMaxX2=max(dMaxX2,tri2[k].X);
						dMinY2=min(dMinY2,tri2[k].Y);
						dMaxY2=max(dMaxY2,tri2[k].Y);
					}
					
					
					if (dMaxX1 <= dMinX2 || dMinX1 >= dMaxX2 ||dMaxY1 <= dMinY2 || dMinY1 >= dMaxY2)
					{
						continue;
					}
					
					GetIntersectionOfTris2(m_AnalysisResultPnts,tri1,tri2);										
				}
			}
			
			else if (nCount)
			{
				offset=j;
				break;
			}			
		}
	}
	return TRUE;	
}


void CGBRVDataSet::GetIntersectionOfTris2(RVLine &pntAry, RVPoint tri1[], RVPoint tri2[])//Sutherland-Hodgeman
{
	RVLine InPutPoly,OutputPoly;
	int i=0,j=0,k=0;
	
	RVLine TriangleVerts;	
	
	for (i=0;i<3;i++)
	{
		InPutPoly.push_back(tri1[i]);
	}
	for (j=0;j<3;j++)
	{
		OutputPoly.clear();
		GetLinesFromTri(OutputPoly,InPutPoly,tri2[j],tri2[j+1]);
		InPutPoly=OutputPoly;
	}
	if (OutputPoly.size()>2)
	{
		TriangulateConvex2(pntAry,OutputPoly);
	}
}

void CGBRVDataSet::GetLinesFromTri(RVLine &OutputPoly, RVLine InPutPoly, RVPoint startPoint, RVPoint endPoint)

int i = 0, j = 0, k=0;
double flag = 0.0;  
RVPoint p1, p2;
RVPoint crossPoint;
InPutPoly.push_back(InPutPoly[0]);


for ( j = 1; j < InPutPoly.size(); j++)  
{     
	p1 = InPutPoly[j-1];  
	flag=LeftOrRight(p1.X,p1.Y,startPoint.X,startPoint.Y,endPoint.X,endPoint.Y);
	p2 = InPutPoly[j];  
	
	if (LeftOrRight(p2.X,p2.Y,startPoint.X,startPoint.Y,endPoint.X,endPoint.Y) == 0)
	{
		OutputPoly.push_back(p2);
		flag=0;
	}
	else if (LeftOrRight(p2.X,p2.Y,startPoint.X,startPoint.Y,endPoint.X,endPoint.Y) <0)
	{  
		if (flag >0)
		{  
			flag = -1;
			GetCrossPnt(p1.X,p1.Y,p2.X,p2.Y,startPoint.X,startPoint.Y,endPoint.X,endPoint.Y,&crossPoint.X,&crossPoint.Y);
			crossPoint.Z=GetZOnLine(crossPoint.X,crossPoint.Y,startPoint,endPoint);
			OutputPoly.push_back(crossPoint);
			
		}  
		OutputPoly.push_back(p2);
	}  
	else
	{  
		if (flag<0)  
		{  
			flag = 1;  
			GetCrossPnt(p1.X,p1.Y,p2.X,p2.Y,startPoint.X,startPoint.Y,endPoint.X,endPoint.Y,&crossPoint.X,&crossPoint.Y);
			crossPoint.Z=GetZOnLine(crossPoint.X,crossPoint.Y,startPoint,endPoint);
			OutputPoly.push_back(crossPoint);
		}  
	}  
}  
}

int CGBRVDataSet::TriangulateConvex2(RVLine &TriangleVerts, RVLine PolyVert)
{
	int nTri=0;
	int i=0;
	
	for (i=1;i<PolyVert.size()-1;i++)
	{
		TriangleVerts.push_back(PolyVert[0]);
		TriangleVerts.push_back(PolyVert[i]);
		TriangleVerts.push_back(PolyVert[i+1]);
		nTri++;
	}	
	return nTri;
	
}

bool CGBRVDataSet::OpenVectorFile(const char *filename)
{

}

//Cohen－Sutherland code
void CGBRVDataSet::Code(double x,double y,int *c,
					   double minx, double miny, double maxx, double maxy) 
{
}


int CGBRVDataSet::LineIntersectRect(double p1x,double p1y,double p2x,double p2y,double minx,
		double miny,double maxx,double maxy,double *x1,double *y1,double *x2,double *y2)
{
}


RVPoint* CGBRVDataSet::GetNewTerrainData(double minLat, double minLong, double maxLat, double maxLong)
{

}


double CGBRVDataSet::GetPointHeightInTriangle(double x, double y, RVPoint *vp)
{

}


void CGBRVDataSet::LineInterpolating(double x1,double y1,double x2,double y2,RVPoint *vp,
								 std::vector<RVPoint> &ptAry)
{

}

void CGBRVDataSet::Simplification(double tolerance,double raise,std::vector<RVPoint> &ptAry,std::vector<bool> &bSel)
{

}

void CGBRVDataSet::Simplification(double tolerance,double raise, std::vector<RVPoint> &ptAry,
								  std::vector<bool> &bSel,int startpos,int endpos)
{

}


void CGBRVDataSet::EliminateCover(double raise,std::vector<RVPoint> &ptAry,std::vector<bool> &bSel,int startpos,int endpos)
{

}


bool CGBRVDataSet::LineRect(double x1, double y1, double x2, double y2, 
							 double rtxMin, double rtyMin, double rtxMax, double rtyMax)
{
	return true;
}


bool CGBRVDataSet::LineIntersectLine(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double *x, double *y)
{

	double cross1 = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
	double cross2 = (x2-x1)*(y4-y1) - (y2-y1)*(x4-x1);
	if(cross1 * cross2 > 0)	
		return false;


	double cross3 = (x4-x3)*(y1-y3) - (y4-y3)*(x1-x3);
	double cross4 = (x4-x3)*(y2-y3) - (y4-y3)*(x2-x3);
	if(cross3 * cross4 > 0)
		return false;

	if (x1 == x2)
	{
		if(x3 == x4) return false;
		double k34 = (y4-y3)/(x4-x3);
		*x = x1;
		*y = k34 * (*x-x3) + y3;
		return true;
	}
	else if (x3 == x4)
	{
		if(x1==x2) return false;
		double k12 = (y2-y1)/(x2-x1);
		*x = x3;
		*y = k12 * (*x-x1) + y1;
		return true;
	}
	else
	{
		double k12 = (y2-y1)/(x2-x1);
		double k34 = (y4-y3)/(x4-x3);
		*x = ((y3 - k34 * x3) - (y1 - k12 * x1)) / (k12 - k34);
		*y = k12 * (*x) + y1 - k12 * x1;
		return true;
	}
}

bool CGBRVDataSet::LineIntersectLine(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double *x, double *y,int *flag)
{
	double cross1 = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
	double cross2 = (x2-x1)*(y4-y1) - (y2-y1)*(x4-x1);

	if(cross1 * cross2 > 0)
		return false;
	
	double cross3 = (x4-x3)*(y1-y3) - (y4-y3)*(x1-x3);
	double cross4 = (x4-x3)*(y2-y3) - (y4-y3)*(x2-x3);
	if(cross3 * cross4 > 0) 
		return false;

	double p=(x2-x1)*(y4-y3)-(y2-y1)*(x4-x3);

	if (cross1<0)
	{
		if (cross4==0)
		{
			return false;
		}
		*flag=1;
	}

	else if(cross1>0)
	{
		if (cross3==0)
		{
			return false;
		}
		*flag=-1;	
	}

	else if(cross1==0)
	{
		if (cross2<0)
		{
			if (cross3==0)
			{
				return false;
			}
			*flag=-1;
		}
		else if (cross2>0)
		{
			if (cross4==0)
			{
			 	return false;
			}
			*flag=1;	
		}
		else if (cross2==0)
		{
			return false;
		}
	}

	if (x1 == x2)
	{
		if(x3 == x4) return false;
		double k34 = (y4-y3)/(x4-x3);
		*x = x1;
		*y = k34 * (*x-x3) + y3;
		return true;
	}
	else if (x3 == x4)
	{
		if(x1==x2) return false;
		double k12 = (y2-y1)/(x2-x1);
		*x = x3;
		*y = k12 * (*x-x1) + y1;
		return true;
	}
	else
	{
		double k12 = (y2-y1)/(x2-x1);
		double k34 = (y4-y3)/(x4-x3);
		*x = ((y3 - k34 * x3) - (y1 - k12 * x1)) / (k12 - k34);
		*y = k12 * (*x) + y1 - k12 * x1;
		return true;
	}
}

void CGBRVDataSet::GetLinesFromRect(std::vector<RVRenderData> &ldtAry, 
									double minLat, double minLong, double maxLat, double maxLong)
{

}

void CGBRVDataSet::GetLinesFromRect(std::vector<RVRenderData> &ldtAry, std::vector<NodeIndex> &indexAry, 
									double minLat, double minLong, double maxLat, double maxLong)
{

}

void CGBRVDataSet::GetPolygonsFromRect(std::vector<RVRenderData> &ldtAry, std::vector<RVPolygon> &plgAry, double minLat, double minLong, double maxLat, double maxLong)
{

}


void CGBRVDataSet::GetPointsFromRect(std::vector<RVRenderData> &ptAry, double minLat, double minLong, double maxLat, double maxLong)
{

}


double CGBRVDataSet::DisPointToLine(double x, double y, double x1, double y1, double x2, double y2)
{

}


void CGBRVDataSet::DouglasLine(float tolerance, std::vector<double> &points,
							   std::vector<bool> &bSel, int startpos, int endpos)
{

}

bool CGBRVDataSet::DouglasPolygon(float tolerance, const std::vector<double> &points, std::vector<double> &output)
{

}

void CGBRVDataSet::Simplify2D(float tolerance)
{

}

RVPoint* CGBRVDataSet::GetPolarTerrainData(double minLat, double minLong, double maxLat, double maxLong)
{

}

void CGBRVDataSet::GetPolarXYPos(double x, double y, double dLatSpan, double *xPos, double *yPos)
{

}

CString CGBRVDataSet::GetFeatureName(OGRFeature *poFeature)
{
	CString strName="";
	strName=poFeature->GetFieldAsString("PINYIN");
	if (!strName.GetLength())
	{
		strName=poFeature->GetFieldAsString("NAME");
	}
	if (!strName.GetLength())
	{
		strName=poFeature->GetFieldAsString("COUNTRY");
	}
	return strName;
}



void CGBRVDataSet::InsertNewLine(std::vector<double> p1, RVPoint *p2, std::vector<RVPointFlag> &p3, std::vector<RVPointFlag> &p4)
{
	int i,j;
	int m,n;
	int count;
	std::vector<RVPointFlag> InOut;
	RVPointFlag tempPnt;
	double alt=0;
	
	ct3=0;

	p3.clear();
	p4.clear();


	for(i=0;i<p1.size()-2;i+=2)
	{
		count=0;
		InOut.clear();

		if (PointInTriangle(p1[i],p1[i+1],p2[0].X, p2[0].Y,p2[1].X, p2[1].Y,p2[2].X, p2[2].Y))
		{
			alt=GetZInArea(p1[i],p1[i+1],p2[0],p2[1],p2[2]);
		}
		p3.push_back(RVPointFlag(p1[i],p1[i+1],alt,0));
		ct3++;
		
		for(j=0;j<3;j++)
		{	
			double outx = 0,outy = 0;
			int flag=0;

			if(LineIntersectLine(p1[i],p1[i+1], p1[i+2],p1[i+3], p2[j].X, p2[j].Y, 
					p2[j+1].X, p2[j+1].Y, &outx, &outy,&flag))
			{
				alt=GetZOnLine(outx,outy,p2[j],p2[j+1]);
				InOut.push_back(RVPointFlag(outx, outy, alt,flag));
				count++;
			}
		}
		
		if(p1[i]<p1[i+2])
		{
			for(m=0;m<count;m++)
			{
				for(n=m+1;n<count;n++)
				{
					if(InOut[m].X>InOut[n].X)
					{
						tempPnt=InOut[m];
						InOut[m]=InOut[n];
						InOut[n]=tempPnt;
					}
				}
			}
		}
		else if(p1[i]>p1[i+2])
		{
			for(m=0;m<count;m++)
			{
				for(n=m+1;n<count;n++)
				{
					if(InOut[m].X<InOut[n].X)
					{
						tempPnt=InOut[m];
						InOut[m]=InOut[n];
						InOut[n]=tempPnt;
					}
				}
			}
		}
		else//p1[i]==p1[i+2]  20140609
		{
			if(p1[i+1]<p1[i+3])
			{
				for(m=0;m<count;m++)
				{
					for(n=m+1;n<count;n++)
					{
						if(InOut[m].Y>InOut[n].Y)
						{
							tempPnt=InOut[m];
							InOut[m]=InOut[n];
							InOut[n]=tempPnt;
						}
					}
				}
			}
			else if(p1[i+1]>p1[i+3])
			{
				for(m=0;m<count;m++)
				{
					for(n=m+1;n<count;n++)
					{
						if(InOut[m].Y<InOut[n].Y)
						{
							tempPnt=InOut[m];
							InOut[m]=InOut[n];
							InOut[n]=tempPnt;
						}
					}
				}
			}
		}
		
		for(m=0;m<count;m++)
		{
			p3.push_back(InOut[m]);
			ct3++;
		}
	}
	
	ct4=0;
	for(i=0;i<3;i++)
	{
		count=0;
		InOut.clear();

		p4.push_back(RVPointFlag(p2[i].X,p2[i].Y,p2[i].Z,0));
		ct4++;
		
		for(j=0;j<p1.size()-2;j+=2)
		{
			double outx = 0,outy = 0;
			int flag=0;
			if(LineIntersectLine(p1[j],p1[j+1], p1[j+2],p1[j+3], p2[i].X, p2[i].Y, 
				p2[i+1].X, p2[i+1].Y, &outx, &outy,&flag))
			{
				alt=GetZOnLine(outx,outy,p2[i],p2[i+1]);
				InOut.push_back(RVPointFlag(outx, outy, alt,flag));	
				count++;
			}
		}
		
		if(p2[i].X<p2[i+1].X)
		{
			for(m=0;m<count;m++)
			{
				for(n=m+1;n<count;n++)
				{
					if(InOut[m].X>InOut[n].X)
					{
						tempPnt=InOut[m];
						InOut[m]=InOut[n];
						InOut[n]=tempPnt;
					}
				}
			}
		}
		else if(p2[i].X>p2[i+1].X)
		{
			for(m=0;m<count;m++)
			{
				for(n=m+1;n<count;n++)
				{
					if(InOut[m].X<InOut[n].X)
					{
						tempPnt=InOut[m];
						InOut[m]=InOut[n];
						InOut[n]=tempPnt;
					}
				}
			}
		}
		else//p2[i].X==p2[i+1].X  20140609
		{
			if(p2[i].Y<p2[i+1].Y)
			{
				for(m=0;m<count;m++)
				{
					for(n=m+1;n<count;n++)
					{
						if(InOut[m].Y>InOut[n].Y)
						{
							tempPnt=InOut[m];
							InOut[m]=InOut[n];
							InOut[n]=tempPnt;
						}
					}
				}
			}
			else if(p2[i].Y>p2[i+1].Y)
			{
				for(m=0;m<count;m++)
				{
					for(n=m+1;n<count;n++)
					{
						if(InOut[m].Y<InOut[n].Y)
						{
							tempPnt=InOut[m];
							InOut[m]=InOut[n];
							InOut[n]=tempPnt;
						}
					}
				}
		}
			
		}
		
		for(m=0;m<count;m++)
		{
			p4.push_back(InOut[m]);
			ct4++;
		}
	}

}

bool CGBRVDataSet::OutJd1(std::vector<RVPointFlag> p3,std::vector<RVPointFlag> p4,std::vector<RVPointFlag> &subpoly)
{
	RVPointFlag tempPnt,tempPnt2,testPnt,testPnt2;
	int i=0;
int j=0;
int nMaxSize=p3.size()+p4.size();
int count=0;
ct=0;
m_nSubPolyCount=0;
ctt[0]=0;
subpoly.clear();

command:
	 if(i<ct3)   
	 {   
		testPnt=p3[i];
		if(p3[i].Flag==1)
		{
			p3[i].Flag=0;
			tempPnt=p3[i];
			tempPnt2=p3[i];
			subpoly.push_back(p3[i]);

			count++;
			if (count>nMaxSize)
			{
				return false;
			}

			  ct++;
			  i++;
			  goto Q3;
		  }
		  else
		  {
		  i++;
		  goto command;
		  }
	}
	  else
	  goto End;
			  
				
				  Q3:
				  if(i==ct3)
				  i=0;
				  testPnt=p3[i];
				  if(p3[i].Flag!=-1)
				  {
					  tempPnt2=p3[i];
					  subpoly.push_back(p3[i]);
					  count++;
					if (count>nMaxSize)
					{
						return false;
					}
				  
					ct++;
					i++;
					
					  goto Q3;
					  }
					  else
					  goto Q4;
					  
						Q4:

					  testPnt=p3[i];
					  testPnt2=p4[j];
						for(j=0;j<ct4;j++)
						{
						if(p4[j].X==p3[i].X&&p4[j].Y==p3[i].Y&&p4[j].Flag==p3[i].Flag)
						{
							tempPnt2=p4[j];
							subpoly.push_back(p4[j]);
							count++;
							if (count>nMaxSize)
							{
								return false;
							}
						j++;
						ct++;
						
						  goto Q5;
						  }
						  }
						  
							Q5:
							if(j==ct4)
							j=0;
							testPnt2=p4[j];
							if(p4[j].Flag!=1)
							{
								tempPnt2=p4[j];
								subpoly.push_back(p4[j]);
								count++;
								if (count>nMaxSize)
								{
									return false;
								}
							
							  j++;
							  ct++;
							  
								goto Q5;
								}
								else
								{
									testPnt2=p4[j];
								if(p4[j].X==tempPnt.X&&p4[j].Y==tempPnt.Y&&p4[j].Flag==1)
								{
									m_nSubPolyCount++;
									ctt[m_nSubPolyCount]=ct;	
								
								  i=0;
								  j=0;
								  														  
								  
								  goto command;
								  }
								  else
								  goto Q6;
								  }
								  
									Q6:
									for(i=0;i<ct3;i++)
									{
										testPnt=p3[i];
										 testPnt2=p4[j];
									if(p3[i].X==p4[j].X&&p3[i].Y==p4[j].Y&&p3[i].Flag==p4[j].Flag)
									{
										tempPnt2=p3[i];
									p3[i].Flag=0;
									
									subpoly.push_back(p3[i]);

									count++;
									if (count>nMaxSize)
									{
										return false;
									}
									
									  ct++;
									  i++;
									  
										goto Q3;
										}
										}
										End:   
										
										  {return true;}
}

int CGBRVDataSet::IsAntiClockwise(double x1, double y1, double x2, double y2, double x3, double y3)
{
	double area=Area(x1, y1, x2, y2, x3, y3);
	if (area>0)
	{
		return 1;
	} 
	else if (area<0)
	{
		return -1;
	} 
	else
	{
		return 0;
	}
}


bool CGBRVDataSet::PointInTriangle(double x, double y, double x1, double y1, double x2, double y2, double x3, double y3)
{
	double S = fabsArea(x1,y1, x2,y2,x3,y3);
	double s1,s2,s3;
	s1 = fabsArea(x, y, x1,y1,x2,y2);
	s2 = fabsArea(x, y, x2,y2,x3,y3);;
	s3 = fabsArea(x, y, x3,y3,x1,y1);;
	double cha = s1 + s2+ s3 - S;

	if (cha < 1e-5)
	{
		return true;
	}
	else
	{
		return false;
	}

}

int CGBRVDataSet::Triangulate(std::vector<RVPoint> &TriangleVerts,std::vector<RVPointFlag> SubPolyVert,int nBeginVert,int nEndVert)
{
	int nTri=0;
	int i=0,j=0,k=0;
	int nAntiClockwise=0;
	bool isempty=TriangleVerts.empty();
	TriangleVerts.clear();
	isempty=TriangleVerts.empty();
	
	std::vector<RVPoint> vPnt;

	
	for (i=nBeginVert;i<=nEndVert;i++)
	{
		if (i>nBeginVert)
		{
			if (SubPolyVert[i].X==SubPolyVert[i-1].X&&SubPolyVert[i].Y==SubPolyVert[i-1].Y)
			{
				continue;
			}
		}
		vPnt.push_back(RVPoint(SubPolyVert[i].X,SubPolyVert[i].Y,SubPolyVert[i].Z));
	}
	
	while(vPnt.size()>3)
	{

		i=0;
		for (j=1;j<vPnt.size()-1;j++)
		{
			nAntiClockwise=IsAntiClockwise(vPnt[j-1].X,vPnt[j-1].Y,vPnt[j].X,vPnt[j].Y,vPnt[j+1].X,vPnt[j+1].Y);
			if (nAntiClockwise==1)
			{
				continue;
			}
			else if (nAntiClockwise==0)
			{
				vPnt.erase(vPnt.begin()+j);
				break;

			}
			else if(nAntiClockwise==-1)
			{
				for (k=j+2;k<vPnt.size();k++)
				{
					if (PointInTriangle(vPnt[k].X,vPnt[k].Y,vPnt[j-1].X,vPnt[j-1].Y,vPnt[j].X,vPnt[j].Y,vPnt[j+1].X,vPnt[j+1].Y))
					{						
						break;
					}
				}
				if (k<vPnt.size())
				{
					continue;					
				} 
				else
				{
					TriangleVerts.push_back(RVPoint(vPnt[j-1].X,vPnt[j-1].Y,vPnt[j-1].Z));
					TriangleVerts.push_back(RVPoint(vPnt[j].X,vPnt[j].Y,vPnt[j].Z));
					TriangleVerts.push_back(RVPoint(vPnt[j+1].X,vPnt[j+1].Y,vPnt[j+1].Z));

					if (vPnt.size()<8)
					{
						int ttt=0;
					}

					vPnt.erase(vPnt.begin()+j);
					nTri++;
					break;
				}				
			} 
		}
	}

	j=1;
	TriangleVerts.push_back(RVPoint(vPnt[j-1].X,vPnt[j-1].Y,vPnt[j-1].Z));
	TriangleVerts.push_back(RVPoint(vPnt[j].X,vPnt[j].Y,vPnt[j].Z));
	TriangleVerts.push_back(RVPoint(vPnt[j+1].X,vPnt[j+1].Y,vPnt[j+1].Z));
	nTri++;

	return nTri;

}

int CGBRVDataSet::TriangulateConvex(std::vector<RVPoint> &TriangleVerts,RVLineFlag SubPolyVert/*,int nBeginVert,int nEndVert*/)
{
	int nTri=0;
	int i=0;
	int nAntiClockwise=0;

	std::vector<RVPoint> vPnt;
	
	
	for (i=0;i<SubPolyVert.size();i++)
	{
		if (i>0)
		{
			if (SubPolyVert[i].X==SubPolyVert[i-1].X&&SubPolyVert[i].Y==SubPolyVert[i-1].Y)
			{
				continue;
			}
		}
		vPnt.push_back(RVPoint(SubPolyVert[i].X,SubPolyVert[i].Y,SubPolyVert[i].Z));
	}
	

	for (i=1;i<vPnt.size()-1;i++)
	{
		TriangleVerts.push_back(vPnt[0]);
		TriangleVerts.push_back(vPnt[i]);
		TriangleVerts.push_back(vPnt[i+1]);
		nTri++;
	}	
	return nTri;
	
}

//disjoint and contain
bool CGBRVDataSet::OutJd2(std::vector<RVPointFlag> p3,std::vector<RVPointFlag> p4,std::vector<RVPointFlag> &subpoly)
{
	subpoly.clear();
	int i;
	bool bPntInTri=false,bPntInPoly=false;
	for (i=0;i<ct3;i++)
	{
		bPntInTri=PointInTriangle(p3[i].X,p3[i].Y,p4[0].X,p4[0].Y,p4[1].X,p4[1].Y,p4[2].X,p4[2].Y);
		if (bPntInTri)
		{
			break;
		}
	}
	
	if (bPntInTri)
	{
		m_nSubPolyCount=1;
		ctt[0]=0;
		ctt[1]=ct3;
		for (i=0;i<ct3;i++)
		{
			subpoly.push_back(p3[i]);
		}	
		return true;
	} 
	else
	{
		for (i=0;i<ct4;i++)
		{			
			bPntInPoly=PntInPoly(p4[i].X,p4[i].Y,p3,ct3);			
			if (!bPntInPoly)
			{
				break;
			}
		}
		if (bPntInPoly)
		{
			m_nSubPolyCount=1;
			ctt[0]=0;
			ctt[1]=ct4;
			for (i=0;i<ct4;i++)
			{
				subpoly.push_back(p4[i]);
			}
			
		}
		else
		{
			m_nSubPolyCount=0;
		}

		return false;
	}

}

bool CGBRVDataSet::OutJdOfTris2(RVLineFlag p3,RVLineFlag p4,RVLineFlag &subpoly)
{
	subpoly.clear();
	int i=0;
	bool bPntInTri2=false,bPntInTri1=false;
	double dArea1=fabsArea(p3[0].X,p3[0].Y,p3[1].X,p3[1].Y,p3[2].X,p3[2].Y);
	double dArea2=fabsArea(p4[0].X,p4[0].Y,p4[1].X,p4[1].Y,p4[2].X,p4[2].Y);
	if (dArea1<=dArea2)
	{
		bPntInTri2=PointInTriangle(p3[0].X,p3[0].Y,p4[0].X,p4[0].Y,p4[1].X,p4[1].Y,p4[2].X,p4[2].Y);
		if (bPntInTri2)
		{
			for (i=0;i<3;i++)
			{
				subpoly.push_back(p3[i]);
			}	
			return true;
		}
		else
		{
			return false;
		}	
	}
	else
	{
		bPntInTri1=PointInTriangle(p4[0].X,p4[0].Y,p3[0].X,p3[0].Y,p3[1].X,p3[1].Y,p3[2].X,p3[2].Y);
		if (bPntInTri1)
		{
			for (i=0;i<3;i++)
			{
				subpoly.push_back(p4[i]);
			}	
			return true;
		}
		else
		{
			return false;
		}
	}
	
}

bool CGBRVDataSet::PntInPoly(double x, double y, std::vector<RVPointFlag> p3, int nvert)
{
	int i, j;
	bool c = false;
	for (i = 1, j = nvert-1; i < nvert; j = i++) 
	{
		if ( ((p3[i].Y>y) != (p3[j].Y>y)) &&
			(x < (p3[j].X-p3[i].X) * (y-p3[i].Y) / (p3[j].Y-p3[i].Y) + p3[i].X) )
			c = !c;
	}
  return c;

}

bool CGBRVDataSet::GetTrianglesFromRect(std::vector<double> p1,RVPoint *grid,std::vector<RVPoint> &pntAry)
{
	int j=0,k=0;
	RVPoint ptri[4];
	bool bIn = false; 
	
	ptri[0]=grid[0];
	ptri[1]=grid[1];
	ptri[2]=grid[2];
	ptri[3]=grid[0];
	
	std::vector<RVPointFlag> p3,p4,subpoly;
	std::vector<RVPoint> TriangleVerts;

	bool isempty=TriangleVerts.empty();
	
	p3.clear();
	p4.clear();
	subpoly.clear();
	InsertNewLine(p1,ptri,p3,p4);

	if (ct4>3)
	{
		OutJd1(p3,p4,subpoly);
		bIn=true;
	} 
	else
	{
		OutJd2(p3,p4,subpoly);
	}


	for (k=0;k<m_nSubPolyCount;k++)
	{
		Triangulate(TriangleVerts,subpoly,ctt[k],ctt[k+1]-1);
		for (j=0; j<TriangleVerts.size(); j++)
		{
			pntAry.push_back(TriangleVerts[j]);
		}
	}
	
	ptri[0]=grid[1];
	ptri[1]=grid[3];
	ptri[2]=grid[2];
	ptri[3]=grid[1];

	p3.clear();
	p4.clear();
	subpoly.clear();
	
	InsertNewLine(p1,ptri,p3,p4);

	if (ct4>3)
	{
		OutJd1(p3,p4,subpoly);
		bIn=true;
	} 
	else
	{		
		OutJd2(p3,p4,subpoly);
	}


	for (k=0;k<m_nSubPolyCount;k++)
	{
		Triangulate(TriangleVerts,subpoly,ctt[k],ctt[k+1]-1);
		for (j=0; j<TriangleVerts.size(); j++)
		{
			pntAry.push_back(TriangleVerts[j]);
		}
	}

	return bIn;

}

double CGBRVDataSet::GetZOnLine(double x, double y, RVPoint pnt0, RVPoint pnt1)
{
		double dis1 = Distance(x, y, pnt0.X, pnt0.Y);
		double dis2 = Distance(x, y, pnt1.X, pnt1.Y);
		double alt = pnt0.Z * dis2/(dis1+dis2) + pnt1.Z * dis1/(dis1+dis2);
		return alt;
}

double CGBRVDataSet::GetZInArea(double x, double y, RVPoint pnt0, RVPoint pnt1, RVPoint pnt2)
{

	double S102 = fabsArea(pnt1.X, pnt1.Y, pnt0.X, pnt0.Y, pnt2.X, pnt2.Y);
	double s1,s2,s3;
	s1 = fabsArea(x, y, pnt1.X, pnt1.Y, pnt0.X, pnt0.Y);
	s2 = fabsArea(x, y, pnt2.X, pnt2.Y, pnt1.X, pnt1.Y);
	s3 = fabsArea(x, y, pnt0.X,pnt0.Y, pnt2.X, pnt2.Y);

	double alpha = s1 / S102;
	double beta = s2 / S102;
	double gama = 1 - alpha - beta;
	double alt = alpha * pnt2.Z + beta * pnt0.Z + gama * pnt1.Z;
	 return alt;
}

void CGBRVDataSet::ComputeGridValues(CVWSceneViewer *pSceneViewer)
{

}

void CGBRVDataSet::SetTerrainAccessor(CVWTerrainAccessor *pTerrainAccessor)
{

}

CVWTerrainAccessor * CGBRVDataSet::GetTerrainAccessor()
{

}

BOOL CGBRVDataSet::Initialize(CVWSceneViewer *pSceneViewer)
{
	m_bInitialized = TRUE;
	return TRUE;
}

void CGBRVDataSet::Update(CVWSceneViewer *pSceneViewer)
{
	if (!m_bInitialized)
	{
		Initialize(pSceneViewer);
	}	
}


BOOL CGBRVDataSet::PerformSelectionAction(CVWSceneViewer *pSceneViewer)
{
	return FALSE;
}

void CGBRVDataSet::Render(CVWSceneViewer *pSceneViewer)
{
	if ((!m_bShp)&&(!m_bVectorAnalysis))
	{
		return;
	}
	CVWDevice* pDevice = pSceneViewer->m_pDevice;
	CVWCameraBase* pCamera = pSceneViewer->m_pCamera;


	
	D3DXMATRIX WorldMatrix;
	D3DXMatrixIdentity(&WorldMatrix);
	D3DXMatrixTranslation(&WorldMatrix,
		(float)-pCamera->m_ReferenceCenter.X,
		(float)-pCamera->m_ReferenceCenter.Y,
		(float)-pCamera->m_ReferenceCenter.Z);
	pDevice->m_pIDirect3DDevice->SetTransform(D3DTS_WORLD,&WorldMatrix);

	DWORD dw1;
	pSceneViewer->m_pDevice->m_pIDirect3DDevice->GetTextureStageState(0, D3DTSS_COLOROP, &dw1);
	pSceneViewer->m_pDevice->m_pIDirect3DDevice->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_DISABLE);
	pSceneViewer->m_pDevice->m_pIDirect3DDevice->SetRenderState(D3DRS_ZENABLE, TRUE);		
	pSceneViewer->m_pDevice->m_pIDirect3DDevice->SetFVF(CUSTOMVERTEXPC_FVF);
	

	pSceneViewer->m_pDevice->m_pIDirect3DDevice->SetRenderState(D3DRS_DEPTHBIAS,0.0004);
	pSceneViewer->m_pDevice->m_pIDirect3DDevice->SetRenderState(D3DRS_SLOPESCALEDEPTHBIAS, 5.0);
	pSceneViewer->m_pDevice->m_pIDirect3DDevice->Clear(0,NULL,D3DCLEAR_ZBUFFER,0,1.0f,0);
	
	pSceneViewer->m_pDevice->m_pIDirect3DDevice->SetRenderState(D3DRS_CULLMODE,D3DCULL_NONE);
	pSceneViewer->m_pDevice->m_pIDirect3DDevice->SetRenderState(D3DRS_FILLMODE ,D3DFILL_WIREFRAME);

	CustomVertex_PositionColored*	pRenderline=NULL;


	if (m_GeoType==RV_Line||m_GeoType==RV_Polygon)
	{
		pRenderline = new CustomVertex_PositionColored[m_AnalysisResultPnts.size()];
		for (int i=0; i<m_AnalysisResultPnts.size(); i++)
		{
			double	radius = m_dEquatorialRadius + m_AnalysisResultPnts[i].Z *m_dZScale+ m_fDistanceAboveSurface;	//抬高

			D3DXVECTOR3 pointXyz = CVWMath::SphericalToCartesian(m_AnalysisResultPnts[i].Y,m_AnalysisResultPnts[i].X,radius);
			pRenderline[i].x = pointXyz.x;
			pRenderline[i].y = pointXyz.y;
			pRenderline[i].z = pointXyz.z;
			pRenderline[i].Colour = D3DCOLOR_ARGB(255, 255, 0, 0);	
		}

		pDevice->m_pIDirect3DDevice->SetFVF(CUSTOMVERTEXPC_FVF);
		if (m_GeoType==RV_Polygon)
		{
			pSceneViewer->m_pDevice->m_pIDirect3DDevice->DrawPrimitiveUP(D3DPT_TRIANGLELIST,
				m_AnalysisResultPnts.size()/3, pRenderline, sizeof(CustomVertex_PositionColored));			
		}
		
		
		if (m_GeoType==RV_Line)
		{
			pSceneViewer->m_pDevice->m_pIDirect3DDevice->DrawPrimitiveUP(D3DPT_LINESTRIP,
				m_AnalysisResultPnts.size()-1, pRenderline, sizeof(CustomVertex_PositionColored));			
		}
	}

	if (pRenderline != NULL)
	{

		delete []pRenderline;
		pRenderline = NULL;
	}

	pDevice->m_pIDirect3DDevice->SetTransform(D3DTS_WORLD,&(pCamera->m_WorldMatrix));
	pSceneViewer->m_pDevice->m_pIDirect3DDevice->SetTextureStageState(0, D3DTSS_COLOROP, dw1);
}

void CGBRVDataSet::GetIntersectionOfTris(RVLine &pntAry, RVPoint tri1[4], RVPoint tri2[4])
{

}



bool CGBRVDataSet::InsertNewLineTris(RVPoint *tri1, RVPoint *tri2, RVLineFlag &p3, RVLineFlag &p4)
{
	ct3=0;
	int i,j;
	int m,n;
	int count;
	int count2=0;
	std::vector<RVPointFlag> InOut;
	RVPointFlag tempPnt;
	RVPoint tempPnt2;
	double alt=0;

	p3.clear();
	p4.clear();


	
	for(i=0;i<3;i++)
	{
		count=0;
		InOut.clear();

		p3.push_back(RVPointFlag(tri1[i].X,tri1[i].Y,tri1[i].Z,0)); 
		ct3++;
		
		for(j=0;j<3;j++)
		{	
			double outx = 0,outy = 0;
			int flag=0;
			bool bnode;

			if(LineIntersectLineTris(tri2[j].X, tri2[j].Y, tri2[j+1].X, tri2[j+1].Y,
				tri1[i].X,tri1[i].Y, tri1[i+1].X,tri1[i+1].Y,  &outx, &outy,&flag, bnode))

			{
				if (flag==0)
				{
					int nNextj=j+2;
					if (nNextj>2)
					{
						nNextj-=3;
					}
					int nNexti=i+2;
					if (nNexti>2)
					{
						nNexti-=3;
					}

					for (int k=0;k<3;k++)
					{
						tempPnt2=tri1[k];
						tempPnt2=tri2[k];
					}

					double cross1=LeftOrRight(tri2[j].X, tri2[j].Y,tri1[i].X,tri1[i].Y, tri1[i+1].X,tri1[i+1].Y);
					double cross2=LeftOrRight(tri2[nNextj].X, tri2[nNextj].Y,tri1[i].X,tri1[i].Y, tri1[i+1].X,tri1[i+1].Y);
					double cross3=LeftOrRight(tri2[j].X, tri2[j].Y,tri1[i+1].X,tri1[i+1].Y, tri1[nNexti].X,tri1[nNexti].Y);
					double cross4=LeftOrRight(tri2[nNextj].X, tri2[nNextj].Y,tri1[i+1].X,tri1[i+1].Y, tri1[nNexti].X,tri1[nNexti].Y);


					if (cross2>=0||cross3>=0)
					{
						return false;
					}
					else if (cross1<=0&&cross4>0)
					{	
						flag=1;
					}
					else if (cross1>0&&cross4<=0)
					{
						flag=-1;
					}
					else
					{
						continue;
					}
				}

				alt=GetZOnLine(outx,outy,tri2[j],tri2[j+1]);
				InOut.push_back(RVPointFlag(outx, outy, alt,flag));
				count++;
				if (!bnode)
				{
					count2++;
				}
			}
		}
		
		if(tri1[i].X<tri1[i+1].X)
		{
			for(m=0;m<count;m++)
			{
				for(n=m+1;n<count;n++)
				{
					if(InOut[m].X>InOut[n].X)
					{
						tempPnt=InOut[m];
						InOut[m]=InOut[n];
						InOut[n]=tempPnt;
					}
				}
			}
		}
		else if(tri1[i].X>tri1[i+1].X)
		{
			for(m=0;m<count;m++)
			{
				for(n=m+1;n<count;n++)
				{
					if(InOut[m].X<InOut[n].X)
					{
						tempPnt=InOut[m];
						InOut[m]=InOut[n];
						InOut[n]=tempPnt;
					}
				}
			}
		}
		else//p1[i]==p1[i+2]  20140609
		{
			if(tri1[i].Y<tri1[i+1].Y)
			{
				for(m=0;m<count;m++)
				{
					for(n=m+1;n<count;n++)
					{
						if(InOut[m].Y>InOut[n].Y)
						{
							tempPnt=InOut[m];
							InOut[m]=InOut[n];
							InOut[n]=tempPnt;
						}
					}
				}
			}
			else if(tri1[i].Y>tri1[i+2].Y)
			{
				for(m=0;m<count;m++)
				{
					for(n=m+1;n<count;n++)
					{
						if(InOut[m].Y<InOut[n].Y)
						{
							tempPnt=InOut[m];
							InOut[m]=InOut[n];
							InOut[n]=tempPnt;
						}
					}
				}
			}
		}
		
		for(m=0;m<count;m++)
		{
			p3.push_back(InOut[m]);
			ct3++;
		}
	}
	
	ct4=0;
	for(i=0;i<3;i++)
	{
		count=0;
		InOut.clear();

		p4.push_back(RVPointFlag(tri2[i].X,tri2[i].Y,tri2[i].Z,0));
		ct4++;
		
		for(j=0;j<3;j++)
		{
			double outx = 0,outy = 0;
			int flag=0;
			bool bnode;
			if(LineIntersectLineTris(tri2[i].X, tri2[i].Y, tri2[i+1].X, tri2[i+1].Y,
				tri1[j].X,tri1[j].Y, tri1[j+1].X,tri1[j+1].Y,  &outx, &outy,&flag,bnode))
			{
				if (flag==0)
				{
					int nNextj=j+2;
					if (nNextj>2)
					{
						nNextj-=3;
					}
					int nNexti=i+2;
					if (nNexti>2)
					{
						nNexti-=3;
					}
					double cross1=LeftOrRight(tri2[i].X, tri2[i].Y,tri1[j].X,tri1[j].Y, tri1[j+1].X,tri1[j+1].Y);
					double cross2=LeftOrRight(tri2[nNexti].X, tri2[nNexti].Y,tri1[j].X,tri1[j].Y, tri1[j+1].X,tri1[j+1].Y);
					double cross3=LeftOrRight(tri2[i].X, tri2[i].Y,tri1[j+1].X,tri1[j+1].Y, tri1[nNextj].X,tri1[nNextj].Y);
					double cross4=LeftOrRight(tri2[nNexti].X, tri2[nNexti].Y,tri1[j+1].X,tri1[j+1].Y, tri1[nNextj].X,tri1[nNextj].Y);
		
					if (cross2>=0||cross3>=0)
					{
						return false;
					}
					else if (cross1<=0&&cross4>0)
					{	
						flag=1;
					}
					else if (cross1>0&&cross4<=0)
					{
						flag=-1;
					}
					else
					{
						continue;
					}

				}
				alt=GetZOnLine(outx,outy,tri2[i],tri2[i+1]);
				InOut.push_back(RVPointFlag(outx, outy, alt,flag));	
				count++;
			}
		}
		
		if(tri2[i].X<tri2[i+1].X)
		{
			for(m=0;m<count;m++)
			{
				for(n=m+1;n<count;n++)
				{
					if(InOut[m].X>InOut[n].X)
					{
						tempPnt=InOut[m];
						InOut[m]=InOut[n];
						InOut[n]=tempPnt;
					}
				}
			}
		}
		else if(tri2[i].X>tri2[i+1].X)
		{
			for(m=0;m<count;m++)
			{
				for(n=m+1;n<count;n++)
				{
					if(InOut[m].X<InOut[n].X)
					{
						tempPnt=InOut[m];
						InOut[m]=InOut[n];
						InOut[n]=tempPnt;
					}
				}
			}
		}
		else//p2[i].X==p2[i+1].X  20140609
		{
			if(tri2[i].Y<tri2[i+1].Y)
			{
				for(m=0;m<count;m++)
				{
					for(n=m+1;n<count;n++)
					{
						if(InOut[m].Y>InOut[n].Y)
						{
							tempPnt=InOut[m];
							InOut[m]=InOut[n];
							InOut[n]=tempPnt;
						}
					}
				}
			}
			else if(tri2[i].Y>tri2[i+1].Y)
			{
				for(m=0;m<count;m++)
				{
					for(n=m+1;n<count;n++)
					{
						if(InOut[m].Y<InOut[n].Y)
						{
							tempPnt=InOut[m];
							InOut[m]=InOut[n];
							InOut[n]=tempPnt;
						}
					}
				}
		}
			
		}
		
		for(m=0;m<count;m++)
		{
			p4.push_back(InOut[m]);
			ct4++;
		}
	}
	return true;
}

bool CGBRVDataSet::OutJdOfTris3(RVPoint tri1[4], RVPoint tri2[4], RVLine &intersection)
{
	int i=0;
	bool bPntInTri2[3],bPntInTri1[3];
	double dArea1=fabsArea(tri1[0].X,tri1[0].Y,tri1[1].X,tri1[1].Y,tri1[2].X,tri1[2].Y);
	double dArea2=fabsArea(tri2[0].X,tri2[0].Y,tri2[1].X,tri2[1].Y,tri2[2].X,tri2[2].Y);
	if (dArea1<=dArea2)
	{
		for (i=0;i<3;i++)
		{
			bPntInTri2[i]=PointInTriangle(tri1[i].X,tri1[i].Y,tri2[0].X,tri2[0].Y,tri2[1].X,tri2[1].Y,tri2[2].X,tri2[2].Y);
		}
	
		if (bPntInTri2[0]&&bPntInTri2[1]&&bPntInTri2[2])
		{
			for (i=0;i<3;i++)
			{
				intersection.push_back(tri1[i]);
			}	
			return true;
		}

		else
		{
			return false;
		}	
	}
	else
	{
		for (i=0;i<3;i++)
		{
			bPntInTri1[i]=PointInTriangle(tri2[i].X,tri2[i].Y,tri1[0].X,tri1[0].Y,tri1[1].X,tri1[1].Y,tri1[2].X,tri1[2].Y);
		}
		
		if (bPntInTri1[0]&&bPntInTri1[1]&&bPntInTri1[2])
		{
			for (i=0;i<3;i++)
			{
				intersection.push_back(tri2[i]);
			}	
			return true;
		}
		else
		{
			return false;
		}
	}
}

bool CGBRVDataSet::LineIntersectLineTris(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double *x, double *y, int *flag, bool &bnode)
{

	if (x1==x3&&y1==y3)
	{
		return false;
	}
	else if ((x1==x4&&y1==y4)||(x2==x3&&y2==y3))
	{
		return false;
	}
	else if (x2==x4&&y2==y4)
	{
		*x=x2;
		*y=y2;
		*flag=0;
		bnode=true;
		return true;
	}	
 	bnode=false;


	double cross1 = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
	double cross2 = (x2-x1)*(y4-y1) - (y2-y1)*(x4-x1);

	if(cross1 * cross2 > 0)
		return false;
	
	double cross3 = (x4-x3)*(y1-y3) - (y4-y3)*(x1-x3);
	double cross4 = (x4-x3)*(y2-y3) - (y4-y3)*(x2-x3);

	if(cross3 * cross4 > 0) //同侧，不交
		return false;

	if ((fabs(cross1)<1e-5||fabs(cross2)<1e-5||fabs(cross3)<1e-5||fabs(cross4)<1e-5)&&cross1*cross2*cross3*cross4!=0)
	{
		int ttt=0;
	}

	if (fabs(cross1)<1e-13)
	{
		cross1=0;
	}
	if (fabs(cross2)<1e-13)
	{
		cross2=0;
	}
	if (fabs(cross3)<1e-13)
	{
		cross3=0;
	}
	if (fabs(cross4)<1e-13)
	{
		cross4=0;
	}
	
	if (cross1<0)
	{
		if (cross3==0)
		{
			return false;
		}
		*flag=-1;
	}
	else if(cross1>0)
	{
		if (cross4==0||cross2==0)
		{
			return false;
		}
		*flag=1;	
	}
	else if(cross1==0)
	{
		if (cross2<0)
		{
			if (cross4==0)
			{
				return false;
			}
			*flag=1;
		}
		else if (cross2>0)
		{
			return false;
		}
		else if (cross2==0)
		{
			return false;
		}
	}
	
	if (x1 == x2) 
	{
		if(x3 == x4) return false;
		double k34 = (y4-y3)/(x4-x3);
		*x = x1;
		*y = k34 * (*x-x3) + y3;
		return true;
	}
	else if (x3 == x4)
	{
		if(x1==x2) return false;
		double k12 = (y2-y1)/(x2-x1);
		*x = x3;
		*y = k12 * (*x-x1) + y1;
		return true;
	}
	else
	{
		double k12 = (y2-y1)/(x2-x1);
		double k34 = (y4-y3)/(x4-x3);
		*x = ((y3 - k34 * x3) - (y1 - k12 * x1)) / (k12 - k34);
		*y = k12 * (*x) + y1 - k12 * x1;
		return true;
	}
}

double CGBRVDataSet::LeftOrRight(double x, double y, double x1, double y1, double x2, double y2)
	double cross1 = (x2-x1)*(y-y1) - (y2-y1)*(x-x1);
	return cross1;
}



bool CGBRVDataSet::GetCrossPnt(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double *x, double *y)
{
	double cross1 = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
	double cross2 = (x2-x1)*(y4-y1) - (y2-y1)*(x4-x1);
	double cross3 = (x4-x3)*(y1-y3) - (y4-y3)*(x1-x3);
	double cross4 = (x4-x3)*(y2-y3) - (y4-y3)*(x2-x3);

	if (x1 == x2)
	{
		if(x3 == x4) return false;
		double k34 = (y4-y3)/(x4-x3);
		*x = x1;
		*y = k34 * (*x-x3) + y3;
		return true;
	}
	else if (x3 == x4)
	{
		if(x1==x2) return false;
		double k12 = (y2-y1)/(x2-x1);
		*x = x3;
		*y = k12 * (*x-x1) + y1;
		return true;
	}
	else
	{
		double k12 = (y2-y1)/(x2-x1);
		double k34 = (y4-y3)/(x4-x3);
		*x = ((y3 - k34 * x3) - (y1 - k12 * x1)) / (k12 - k34);
		*y = k12 * (*x) + y1 - k12 * x1;
		return true;
	}

}
