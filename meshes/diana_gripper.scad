/** simple four-servo two-finger gripper with mountplate for
 *  the Agile Robots Diana7 robot.
 *
 * 2022.07.01 - export first meshes (v1)
 * 2022.07.01 - created (based on feetech-servo-hand.scad)
 *
 * (c) 2021, 2022 fnh, hendrich@informatik.uni-hamburg.de
 */


fn = 100;
eps = 0.01;
M6_bore = 6.2;
M4_bore = 4.2;

ddx = 75/2 + 0.2;
ddy = 35;
ddz = 15 + 5 + 12.7/2;

make_diana_flange = 1;
make_gripper_base = 1;
make_palm_plate   = 1;
make_servo_base1  = 1; // first finger
make_connector1   = 1;

make_servo_base2  = 1; // second finger
make_connector2   = 1;

show_loadcells    = 1;

show_servo1P      = 1; // first finger, proximal
show_servo1D      = 1; // first finger, distal
show_bracket1     = 1; // first finger, distal mounting bracket
show_digit1       = 1; // first finger, DIGIT sensor

show_servo2P      = 1; // second finger, proximal
show_servo2D      = 1; // second finger, distal
show_bracket2     = 1; // second finger, distal mounting bracket
show_digit2       = 1; // second finger, DIGIT sensor

if (make_diana_flange) diana_flange();
if (make_gripper_base) gripper_base();
if (make_palm_plate)   translate( [0,0, 15+5+12.7] ) palm_plate();

if (make_servo_base1)  translate( [75.0-6-5,0,20] ) servo_base();
if (make_servo_base2)  translate( [-75.0+6+5,0,20] ) servo_base();
               


if (show_loadcells) {
translate( [ ddx,  ddy, ddz] ) load_cell_komputer_10kg();   
translate( [-ddx,  ddy, ddz] ) load_cell_komputer_10kg();   
translate( [ ddx, -ddy, ddz] ) load_cell_komputer_10kg();   
translate( [-ddx, -ddy, ddz] ) load_cell_komputer_10kg();   
}





if (show_servo1P) {
translate( [75-10,20,50] ) rotate( [0,90,90] ) feetech_scs20_servo( extrude_21mm_axles=0);
translate( [75-10,20+6,50] ) rotate( [0,90,-90] ) feetech_servo_disc( extrude_screws_length=0, extrude_screws_head=[] );
// translate( [75-10,-20-6,50] ) rotate( [0,90,90] ) feetech_servo_disc( extrude_screws_length=0, extrude_screws_head=[] );
}

if (make_connector1) 
translate( [45,1,50] ) rotate( [-90,0,-90] ) // for complete model
// rotate( [-90,0,-90] ) translate( [0,0,-20] ) // for standalone rendering
proximal_medial_connector();

if (show_servo1D) {
translate( [75-10-47,20,50] ) rotate( [-90,0,0] ) feetech_scs20_servo( extrude_21mm_axles=0);
translate( [75-10-47,20+6,50] ) rotate( [0,90,-90] ) feetech_servo_disc( extrude_screws_length=0, extrude_screws_head=[] );
translate( [75-10-47,-20-6,50] ) rotate( [0,90,90] ) feetech_servo_disc( extrude_screws_length=0, extrude_screws_head=[] );
}

if (show_bracket1) 
// translate( [18,0,15+5+30+27.5] ) rotate( [0,180,90] ) 
feetech_scs_bracket_big();

if (show_digit1) 
translate( [ 18,0,90] ) rotate( [0,0,180]) 
digit_sensor();




if (show_servo2P) {
translate( [-(75-10),20,50] ) rotate( [0,90,90] ) feetech_scs20_servo( extrude_21mm_axles=0);
translate( [-(75-10),20+6,50] ) rotate( [0,90,-90] ) feetech_servo_disc( extrude_screws_length=0, extrude_screws_head=[] );
// translate( [-(75-10),-20-6,50] ) rotate( [0,90,90] ) feetech_servo_disc( extrude_screws_length=0, extrude_screws_head=[] );
}

if (make_connector2) 
translate( [-45,1,50] ) rotate( [90,0,-90] ) proximal_medial_connector();

if (show_servo2D) {
translate( [-(75-10-47),20,50] ) rotate( [-90,180,0] ) feetech_scs20_servo( extrude_21mm_axles=0);
translate( [-(75-10-47),20+6,50] ) rotate( [0,90,-90] ) feetech_servo_disc( extrude_screws_length=0, extrude_screws_head=[] );
// translate( [-(75-10-47),-20-6,50] ) rotate( [0,90,90] ) feetech_servo_disc( extrude_screws_length=0, extrude_screws_head=[] );
}

if (show_bracket2) 
translate( [-18,0,15+5+30+27.5] ) rotate( [0,180,90] ) feetech_scs_bracket_big();

if (show_digit2) {
translate( [-18,-20,90] ) digit_sensor();
translate( [-18,+20,90] ) digit_sensor();
}

/**
 * hardcoded size palm plate, mounts on load-cell screws
 */
module palm_plate( 
  alpha = 0.9,
  dx = 45, dy = 85, dz = 3, dx2=90,
  load_cell = [75.0, 12.7, 12.7],
  bore_offsets = [-16, -6, 6, 16], // relative to center of palm
  M4_bore=4.2, M4_head_diameter = 7.0+1.0, M4_head_length=4+1.0, M4_screw_length=30, 
)
{
  difference() {
    union() {
      translate( [0,0,dz/2] ) 
        cube( [dx, dy, dz], center=true );
      translate( [0,0,dz/2] ) 
        cube( [dx2, dy-2*15, dz], center=true );
    }
    
     // loadcell mounting bores
    for( ddy=[-35, +35] ) { // loadcell y positions
      for( ddx=bore_offsets ) { // loadcell M4 bores
        translate( [ddx, ddy, -dz-eps] ) 
          cylinder( d=M4_bore, h=dz+ddz+2*eps, center=false, $fn=50 );
        translate( [ddx, ddy, -dz-eps] ) 
          cylinder( d=M4_head_diameter, h=M4_head_length, center=false, $fn=50 );
      }
    }

  }
}


module servo_base(
  alpha = 0.9,
  dx = 6+10+6, dy = 85, dz = 15, ddy = 35,
  dty = 2.5, // thickness of servo mounting "towers"
  servoy = 41.1, // some SCservos are slightly wider than 40.0
  load_cell = [75.0, 12.7, 12.7],
  bore_offsets = [-5, 5], // relative to center of servo_base block!
  M4_bore=4.2, M4_head_diameter = 7.0+1.0, M4_head_length=4+1.0, M4_screw_length=30, 
)
{
  difference() {
    union() {
      // main mounting block
      color( "gold", alpha )
      translate( [0,0,-dz/2] )
        cube( [dx,dy,dz], center=true );
      
      // servo mounting "towers"
      color( "gold", alpha )
      for( ddty=[-servoy/2-dty/2, servoy/2+dty/2] ) 
        translate( [0, ddty,10] )
          cube( [dx, dty, 20], center=true );      
    }
    
    // loadcell mounting bores
    for( ddy=[-35, +35] ) { // loadcell y positions
      for( ddx=[-5, 5] ) { // loadcell M4 bores
        translate( [ddx, ddy, -dz-eps] ) 
          cylinder( d=M4_bore, h=dz+ddz+2*eps, center=false, $fn=50 );
        translate( [ddx, ddy, -dz-eps] ) 
          cylinder( d=M4_head_diameter, h=M4_head_length, center=false, $fn=50 );
      }
    }
    
    // servo mounting bores
    translate( [0,20,30-eps] ) rotate( [0,90,90] ) feetech_scs20_servo( extrude_mounting_screws=dty, extrude_mounting_screws_heads=4 
    );
  }
}


module gripper_base(
  alpha = 0.9,
  dx = 90, dy = 85, dz = 15,
  M6_bore=6.2, M6_head_diameter = 10.0+1.0, M6_head_length=7+1.0, M6_screw_length=20,
  M4_bore=4.2, M4_head_diameter = 7.0+1.0, M4_head_length=4+1.0, M4_screw_length=30,
)
{
  difference() {
    color( "lightgreen", alpha )
    union() {
      // mounting plate for Diana7 flange
      translate( [0,0,dz/2] )
        cube( [dx,dy,dz], center=true );
      
      // extra blocks to mount loadcells, min z-height 3mm for loadcell clearance
      ddz = 5;
      for( dyy =[dy/2-16/2,-dy/2+16/2] )
        translate( [0, dyy,dz+ddz/2] )
          cube( [23+23, 16, ddz], center=true );
    }
    
    // Diana7 M6 D912 screw bores, max length in flange is 12 
    rr = 25.0; 
    for( phi=[0,90,180,270] ) {
      rotate( [0,0,phi+45] ) translate( [rr,0,-eps] )
        cylinder( d=M6_bore, h=dz+2*eps, center=false, $fn=40 );
      rotate( [0,0,phi+45] ) translate( [rr,0,dz-M6_head_length] )
        cylinder( d=M6_head_diameter, h=M6_head_length+2*eps, center=false, $fn=40 );
    }
    // Diana7 alignment pin bore, max depth in flange in 6
    translate( [rr,0,-eps] )
      cylinder( d=6.0, h=dz+2*eps, center=false, $fn=40 );
    
    // loadcell mount bores
    for( ddy=[-35, +35] ) { // loadcell y positions
      for( ddx=[-16,-6, 6,16] ) { // loadcell M4 bores
        translate( [ddx, ddy, -eps] ) 
          cylinder( d=M4_bore, h=dz+ddz+2*eps, center=false, $fn=50 );
        translate( [ddx, ddy, -eps] ) 
          cylinder( d=M4_head_diameter, h=M4_head_length, center=false, $fn=50 );
      }
    }
    
  } // difference
}


/* basic CAD model of the Agile Robots Diana-7 robot tool flange.
 * Centered in x and y, flange aligned at z=0.
 * Note: 45-deg bevels 0.1mm on outer and inner cylinders NOT modelled yet.
 */ 
module diana_flange() {
  hh = 70; dd= 85;
  h2 = 6; d2 = 63; d3 = 31.5;
  rr = 25;
  
  color( "silver" )
  translate( [0,0,-hh-h2] )
    cylinder( d=dd, h=hh+eps, center=false, $fn=200 );
  
  color( "gray" )
  difference() {
    translate( [0,0,-h2] ) 
      cylinder( d=d2, h=h2, center=false, $fn=200 );
    translate( [0,0,-h2-eps] ) 
      cylinder( d=d3, h=h2+2*eps, center=false, $fn=200 );

    // screw bores M6x12
    for( phi=[0,90,180,270] ) 
      rotate( [0,0,phi+45] ) translate( [rr,0,-12+eps] )
        cylinder( d=M6_bore, h=12.0, center=false, $fn=40 );

    // alignment pin bore 6H7, depth 6mm
    translate( [rr,0,-6-1.5*eps] )
      cylinder( d=M6_bore, h=6.0+3*eps, center=false, $fn=40 );
  }
}



module proximal_medial_connector() {
  dxmin = 49; // original: 51.5; 
  tt=3.0; // plate thickness 

  qz = -32.0;
  
  bx1 = -20.7; 
  bx2 =  23.1;
  by = 0;
  bz = 20-2;

  extra_proximal_height = 5.0;
  
  
  color( "lightblue" ) 
  translate( [0,0,extra_proximal_height] )
  translate( [0,0,0] ) {
    // v2 tt=2.0
    translate( [-dxmin/2-tt/2,0,0] ) servo_disc_adapter_plate( thickness=tt, inbus_head_cut_depth = 0, servo_disc_cut_depth=0.5 );
    translate( [+dxmin/2+tt/2,0,0] ) bottom_axle_adapter_plate( thickness=tt, make_m3_bores=0, bore=6.05, rim=0 );

    // v1 tt=3.0
    // translate( [-dxmin/2-tt/2,0,0] ) servo_disc_adapter_plate( inbus_head_cut_depth = 1, servo_disk_cut_depth=1.0 );
    // translate( [+dxmin/2+tt/2,0,0] ) bottom_axle_adapter_plate( make_m3_bores=0, bore=16.0 );
            
    difference() {
      union() {
        translate( [0,0,-tt/2] ) cube( [dxmin+2*tt, 20, tt], center=true ); // middle piece
        translate( [bx1,by,-tt-bz/2] ) cube( [tt,20,bz], center=true ); // left bracket
        translate( [bx2,by,-tt-bz/2] ) cube( [tt,20,bz], center=true ); // right bracket
      }  

      translate( [-dxmin/2 + 11/2 - 0.2, 0, qz+0.5] ) rotate( [0,-90,0] ) feetech_scs20_servo();
      translate( [-dxmin/2 + 11/2 + 0.2, 0, qz] ) rotate( [0,-90,0] ) feetech_scs20_servo();
      translate( [-dxmin/2 + 11/2, 0, qz] ) rotate( [0,-90,0] ) feetech_scs20_servo( extrude_mounting_screws=10 );

      translate( [bx1,by,-tt-30] ) rotate( [90,0,90] ) cylinder( d=23.0, h=tt+2*eps, $fn=fn, center=true );
      translate( [bx2,by,-tt-30] ) rotate( [90,0,90] ) cylinder( d=23.0, h=tt+2*eps, $fn=fn, center=true );

      for( dy=[-5,+5] ) {
        for( dx=[-15,-5,5,15] ) {
          translate( [dx,dy,-tt-eps] ) cylinder( d1=3.0, d2=2.6, h=tt+2*eps, center=false, $fn=20 );
        }
      }
    }
  }
} // proximal_medial_connector



use <feetech_servo.scad>
use <load_cells.scad>
use <digit_sensor.scad>
// use <ball_bearings.scad>

