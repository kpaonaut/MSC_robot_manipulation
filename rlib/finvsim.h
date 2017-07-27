// modified by Wenjie Chen, 06/10/2016, to add assignment functions for struct variables

#ifndef	_finvsim_h_
#define	_finvsim_h_

#include	<math.h>

#include	"vmathsim.h"

//#define	TRUE	1
//#define	FALSE	0

/* special type def for coulomb fric*/
typedef struct vector2 {
	double	x, y;
} Vect2;

/* A robot link structure */
typedef struct _link {
	/**********************************************************
	 *************** kinematic parameters *********************
	 **********************************************************/
	double	alpha;		/* link twist */
	double	A;		/* link offset */
	double	D;		/* link length */
	double	theta;		/* link rotation angle */
	double	offset;		/* link coordinate offset */
	//int	sigma; 		/* axis type; revolute or prismatic */

	/**********************************************************
	 ***************** dynamic parameters *********************
	 **********************************************************/

	/**************** of links ********************************/
	Vect	*rbar;		/* centre of mass of link wrt link origin */
	double	m;		/* mass of link */
	double	*I;		/* inertia tensor of link wrt link origin */

	/**************** of actuators *****************************/
		/* these parameters are motor referenced */
	double	Jm;		/* actuator inertia */
	double	G;		/* gear ratio */
	double	B;		/* actuator friction damping coefficient */
	Vect2	*Tc;		/* actuator Coulomb friction coeffient */

} Link;

/* A robot */
//typedef struct _robot {
	//int	njoints;	/* number of joints */
	//Vect	*gravity;	/* gravity vector */
	//DHType	dhtype;		/* kinematic convention */
	//Link	*links;		/* the links */
	//Link	*l1;
	//Link	*l2;
	//Link	*l3;
	//Link	*l4;
	//Link	*l5;
	//Link	*l6;
//} Robot;

void	vect_assign (Vect *a, Vect *b);
void	vect2_assign (Vect2 *a, Vect2 *b);
void	double9_assign (double *a, double *b);
void	link_assign (Link *a, Link *b);

#endif
