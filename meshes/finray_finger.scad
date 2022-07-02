/* finray_finger.scad
 *
 * (c) 2022 fnh, hendrich@informatik.uni-hamburg.de
 */
 
fn = 50;
eps = 0.1;

translate( [0,0,-8] ) 
finray_finger();

function interpolate( p1, p2, alpha ) =
  alpha*p1 + (1-alpha)*p2;
 
module finray_finger(
  h = 80, d = 40, t = 16, r = 0.6,
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
    translate( top ) cylinder( r=r, h=t, center=false, $fn=fn );
    translate( origin ) cylinder( r=r, h=t, center=false, $fn=fn );
  }
  hull() {
    translate( top ) cylinder( r=r, h=t, center=false, $fn=fn );
    translate( back1 ) cylinder( r=r, h=t, center=false, $fn=fn );
  }
  hull() {
    translate( front1 ) cylinder( r=r, h=t, center=false, $fn=fn );
    translate( back1 ) cylinder( r=r, h=t, center=false, $fn=fn );
  }
  
  imax = 9;
  for( i=[1:imax] ) {
    alpha = -0.02 + 0.9*i/imax;
    beta = 1.0*i/imax;
    hull() {
      translate( interpolate( top, front1, alpha )) cylinder( r=r, h=t, center=false, $fn=fn );   
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