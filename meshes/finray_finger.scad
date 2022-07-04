/* finray_finger.scad
 *
 * (c) 2022 fnh, hendrich@informatik.uni-hamburg.de
 */
 
fn = 50;
eps = 0.1;

//translate( [-100,0,-8] ) 
//finray_finger();

//translate( [0,0,-10] ) 
//finray_finger( h=50, h2=0, d=30, t1  = 20, t=20, dovetail=[6,20,12,2] );


//translate( [10,10,0] ) 
//rotate( [0,0,60] ) 
//translate( [0,0,0] ) rotate( [0,90,-90] ) feetech_scs_bracket_big();



// translate( [-30,0,0] ) 
// translate( [0,0,0] ) rotate( [0,90,-90] ) feetech_scs_bracket_big();

// translate( [-30,0,0] ) 
{
  h1 = 5;
  h = 40; t1 = 10; r = 0.7; t = 30;

  front1 = [-10,h1,0];
  back1  = [+10+r,r,0];
  origin = [0,0,0];
  top    = [20,h,0];
  difference() {
    union() {
      translate( [0,h1/2,0] )
        cube( [20+2*r,h1,40], center=true );   
    
      hull() {
        translate( top ) cylinder( r=r, h=t1, center=true, $fn=fn );
        translate( front1 ) cylinder( r=r, h=t, center=true, $fn=fn );
      }
      hull() {
        translate( top ) cylinder( r=r, h=t1, center=true, $fn=fn );
        translate( back1 ) cylinder( r=r, h=t, center=true, $fn=fn );
      }
      
      alpha = [0.25, 0.4, 0.55, 0.7];
      beta  = [0.15, 0.35, 0.6, 0.8];
      for (i= [0:len(alpha)-1]) {
        hull() {
          translate( interpolate( top, front1, alpha[i] )) cylinder( r=r, h=ip(t1,t,alpha[i]), center=true, $fn=fn );   
          translate( interpolate( top, back1, beta[i] )) cylinder( r=r, h=ip(t1,t,beta[i]), center=true, $fn=fn );   
        }
      }

        hull() {
          translate( interpolate( top, front1, 0.13 )) cylinder( r=r, h=ip(t1,t,0.13), center=true, $fn=fn );   
          translate( interpolate( front1, back1, 0.3 )) cylinder( r=r, h=ip(t,t,0.5), center=true, $fn=fn );   
        }
    } // union
      
    for( dx=[-5,+5] ) {
      for( dz=[-15,-5,+5,+15] ) {
        translate( [dx,-0.5,dz] ) rotate( [-90,0,0] ) 
          cylinder( d=3.2, h=5, center=false, $fn=50 );
        translate( [dx,2, dz] ) rotate( [-90,0,0] ) 
          cylinder( d=5.2, h=5-1, center=false, $fn=6 );
      }
    }
  }      
  
  // "nupsies"
  if (make_nupsies)
  translate( [-1,0,0] ) 
  intersection() 
  {
    hull() {
      translate( top ) cylinder( r=r, h=t1, center=true, $fn=fn );
      translate( front1 ) cylinder( r=r, h=t, center=true, $fn=fn );
    }
    union() {
      for( dy=[h1+1:1.5:h] ) {
        for( dz=[-20:1.5:20] ) {
          xx = interpolate( 25-r, -10, (dy-h1)/h );
          echo( dy, dz );
          translate( [xx,dy,dz] )
            sphere( d=1.1, $fn=15 );
            // #cube( [0.5,0.3,0.3], center=true );
        }
      }
    }
  }

  // "ridges"
  if (make_ridges ||  true)
  translate( [-1,0,0] ) 
  intersection() 
  {
    hull() {
      translate( top ) cylinder( r=r, h=t1, center=true, $fn=fn );
      translate( front1 ) cylinder( r=r, h=t, center=true, $fn=fn );
    }
    union() {
      for( dy=[h1+1:1.5:h] ) {
        // for( dz=[-20:1.5:20] ) {
          xx = interpolate( 25-r, -10, (dy-h1)/h );
          dz = interpolate( 0, 40, (dy-h1)/h );
          echo( dy );
          translate( [xx,dy,0] )
            cylinder( d=1, h=4, center=true, $fn=10 );
          translate( [xx,dy,3] )
            cylinder( d=1, h=40, center=false, $fn=10 );
          translate( [xx,dy,-3] )
            rotate( [0,180,0] ) 
            cylinder( d=1, h=40, center=false, $fn=10 );
            // #cube( [0.5,0.3,0.3], center=true );
        // }
      }
    }
  }

}



function interpolate( p1, p2, alpha ) =
  alpha*p1 + (1-alpha)*p2;

function ip( v1, v2, alpha ) =
  alpha*v1 + (1-alpha)*v2;


 
module finray_finger(
  h = 80, d = 40, t1 = 16, t = 16, r = 0.6, 
  h2 = 10, d2 = 10,
  dovetail = [6, 30, 12, 2 ]
)
{
  // main finray-triangle
  top = [0,h,0];
  front1 = [0,h2,0];
  back1  = [d,h2,0];
  origin = [0,0,0];
  
  hull() {
    translate( top ) cylinder( r=r, h=t1, center=false, $fn=fn );
    translate( origin ) cylinder( r=r, h=t, center=false, $fn=fn );
  }
  hull() {
    translate( top ) cylinder( r=r, h=t1, center=false, $fn=fn );
    translate( back1 ) cylinder( r=r, h=t, center=false, $fn=fn );
  }
  hull() {
    translate( front1 ) cylinder( r=r, h=t1, center=false, $fn=fn );
    translate( back1 ) cylinder( r=r, h=t, center=false, $fn=fn );
  }
  
  imax = 9;
  for( i=[1:imax] ) {
    alpha = -0.02 + 0.9*i/imax;
    beta = 1.0*i/imax;
    hull() {
      translate( interpolate( top, front1, alpha )) cylinder( r=r, h=ip(t1,t,alpha), center=false, $fn=fn );   
      translate( interpolate( top, back1, beta )) cylinder( r=r, h=t, center=false, $fn=fn );   
    }
  }
  
  // bottom front part
  hull() {
    translate( origin ) cylinder( r=r, h=t, center=false, $fn=fn );
    translate( front1 ) cylinder( r=r, h=t, center=false, $fn=fn );
    translate( [d2,h2,0] ) cylinder( r=r, h=t, center=false, $fn=fn );
    translate( [d2,0,0] ) cylinder( r=r, h=t, center=false, $fn=fn );
  }

  // dovetail: [height,depth,thickness,inset] 
  hd = dovetail[0];
  dd = dovetail[1];
  td = dovetail[2];
  xx = dovetail[3];
  translate( [d2+dd/2,h2-hd/2, t/2] )
    cube( [dd,hd,td-2*xx], center=true );
  translate( [d2+dd/2, h2-hd+xx/2, t/2] )
    cube( [dd,xx,td], center=true );
}


use <feetech_servo.scad>