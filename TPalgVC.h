#include <math.h>
#ifdef TPALGVC_EXPORTS
#define TPALGVC_API __declspec(dllexport)
#else
#define TPALGVC_API __declspec(dllimport)

#endif



// TPALGVC_API int fnTPalgVC(void);

TPALGVC_API int BL2XY(double arr[4],double B, double L,  double x,  double y, double a, double f, int beltWidth, bool assumedCoord);
TPALGVC_API int XY2BL(double arr[4],double x, double y,  double B,  double L, double a, double f, int beltWidth);
TPALGVC_API double MeridianLength(double B, double a, double f);