/** feetech_servo_hand.scad
 *
 * Basic 2-finger or 3-finger hand using the Feetech SCS-series
 * digital bus servos.
 *
 * Surprisingly, there don't seem to be many implementations of this idea.
 * https://www.hackster.io/news/james-bruton-s-force-controlled-robot-gripper-2b02407aa68b 
 * https://www.robotshop.com/community/robots/show/robotic-servo-grippers-compatible-with-arduino-raspberry-picaxe
 * https://www.semanticscholar.org/paper/Design-of-servo-actuated-robotic-gripper-using-for-Shaw-Dubey/657e72d0716f69bfdbd130a7f36c54a2c402732e
 * DOI:10.1109/ARIS.2016.7886619Corpus ID: 18108336
   Design of servo actuated robotic gripper using force control for range of objects
   J. Shaw, Vipul Dubey Published 2016 Engineering
 * https://tinkersphere.com/raspberry-pi-compatible-components/2322-robotic-hand-kit-servos-included.html
 *
 * 2020.11.30 - fix SCS20 servo size, thicker brackets
 * 2020.11.29 - proximal/medial/elliptic-tip designs
 * 2020.11.21 - created
 *
 * (C) 2020, fnh, hendrich@informatik.uni-hamburg.de
 */


use <feetech_servo.scad>
use <ball_bearings.scad>

fn  = 100;
eps = 0.1;

make_single_components = 1;
make_hand_v1 = 0;

// meshes are called v2, v3, ...
make_finger_v2 = 0;
make_finger_v2_tip = 1;
make_finger_v2_medial = 1;
make_finger_v2_proximal = 1;
make_servo_and_discs = 1;

make_two_finger_gripper_base = 1;
make_two_finger_gripper_fingers = 1;
make_three_finger_gripper = 0;




if (make_single_components) {
// single components
translate( [ 100,200,0] ) cylindrical_fingertip( xsize=40 );
translate( [  50,200,0] ) elliptical_fingertip();
translate( [   0,200,0] ) flat_tactile_matrix();
translate( [ 300,200,0] ) sunrise_M3207_servo_hand_adapter();
translate( [ 200,200,0] ) pa10_servo_hand_adapter();
translate( [-100,200,0] ) feetech_scs15_servo( extrude_mounting_screws=6 );
translate( [-150,200,0] ) ball_bearing_16_8_5( seal=1 );
translate( [-180,200,0] ) ball_bearing_16_6_5( seal=1, rim=1 );
translate( [-300,200,0] ) distance_ring( outer=8.0, inner=6.0, height=5.0 );
translate( [   0,150,0] ) servo_disc_adapter_plate( thickness=2.0, height=25 );
}


if (make_finger_v2) translate( [0,0,49] ) finger_v2();

if (make_two_finger_gripper_base) translate( [0,0,0] ) two_finger_gripper();

if (make_three_finger_gripper) translate( [0,0,0] ) three_finger_gripper();


if (make_hand_v1) {
// first finger
translate( [0,-50,0] ) rotate( [0,0,-15] ) 
{
  translate( [0,0,0] )     basic_two_servo_finger();
  translate( [78,0,-20] )  cylindrical_fingertip();
  translate( [10,11,-20] ) rotate( [-90,0,0] ) flat_tactile_matrix();
  translate( [58,11,-20] ) rotate( [-90,0,0] ) flat_tactile_matrix();
}


// opposing finger
translate( [0,50,0] ) rotate( [0,0,10] ) 
{
  translate( [0,0,0] )       basic_two_servo_finger();
  translate( [78,0,-20] )    elliptical_fingertip();
  translate( [10,0-11,-20] ) rotate( [90,0,0] ) flat_tactile_matrix();
  translate( [58,0-11,-20] ) rotate( [90,0,0] ) flat_tactile_matrix();
}

// possible thumb structure?
translate( [0,-20,-60] ) rotate( [90,4,0] )
{
   
translate( [-25,-20,-20] ) 
  rotate( [-90,0,180] )
    feetech_scs15_servo( extrude_mounting_screws=6 );
  
translate( [0,0,0] )    basic_two_servo_finger();
translate( [78,0,-20] ) elliptical_fingertip();
translate( [10,11,-20] ) rotate( [-90,0,0] ) flat_tactile_matrix();
translate( [58,11,-20] ) rotate( [-90,0,0] ) flat_tactile_matrix();
}
} // if make_hand




module finger_v2() 
{
  dxmin = 49; // original: 51.5; 
  tt=3.0; // plate thickness 

  qz = -32.0;
  
  bx1 = -20.7; 
  bx2 =  23.1;
  by = 0;
  bz = 20-2;

  extra_proximal_height = 5.0;
  
  if (make_finger_v2_tip) 
  translate( [0,0,extra_proximal_height] )
  color( "violet" ) {
    difference() { 
      union() {
        translate( [-dxmin/2+24+1.5,0,45+1] ) rotate( [0,-90,0] ) elliptical_fingertip();
        translate( [-dxmin/2+24+2.0,0,35.8] ) cube( [45,20,22], center=true );
      }
      translate( [-dxmin/2 + 11/2-0.2, 0, 15.2] ) rotate( [0,-90,0] ) 
        feetech_scs20_servo();
      translate( [-dxmin/2 + 11/2-0.2, 0, 15.0] ) rotate( [0,-90,0] ) 
        feetech_scs20_servo( extrude_mounting_screws=20, extrude_21mm_axles=20);
      translate( [-dxmin/2 + 11/2+0.2, 0, 15.0] ) rotate( [0,-90,0] ) 
        feetech_scs20_servo( extrude_mounting_screws=20, extrude_21mm_axles=20);
    }

  }
  
  if (make_finger_v2_medial) // 6.05 bore for bottom servo axle, no ball bearing
  color( "blue" ) 
  translate( [0,0,extra_proximal_height] )
  translate( [0,0,0] ) {
    // v2 tt=2.0
    translate( [-dxmin/2-tt/2,0,0] ) servo_disc_adapter_plate( thickness=tt, inbus_head_cut_depth = 0, servo_disk_cut_depth=0.5 );
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

  if (make_finger_v2_proximal) 
  color( "green" )
  translate( [0,0,-47] ) {
    // v2 tt=2.0
    translate( [-dxmin/2-tt/2,0,0] ) servo_disc_adapter_plate( thickness=tt, inbus_head_cut_depth = 0, servo_disk_cut_depth=0.5, height=30 );
    translate( [+dxmin/2+tt/2,0,0] ) bottom_axle_adapter_plate( thickness=tt, make_m3_bores=0, bore=6.05, rim=0, height=30 );
    
    difference() {
      translate( [0,0,-2/2] ) cube( [dxmin+2*tt, 20, 2], center=true );
      
      // for( dy=[-5,+5] ) {
      for( dy=[0] ) {
        for( dx=[-15,-5,5,15] ) {
          translate( [dx,dy,-tt-eps] ) cylinder( d1=3.0, d2=2.6, h=tt+2*eps, center=false, $fn=20 );
        }
      }
    }
  }
  
  if (make_servo_and_discs && true )   
    translate( [0,0,extra_proximal_height] )
  {
    translate( [-dxmin/2 + 11/2, 0, 15.0] ) rotate( [0,-90,0] ) feetech_scs20_servo( extrude_21mm_axles=0);
    // translate( [dxmin/2 + 2.4, 0, 15.0] )  rotate( [0,-90,0] ) ball_bearing_16_8_5( rim=1 );
    translate( [-dxmin/2 - 0.8, 0, 15] ) rotate( [0,90,0] ) feetech_servo_disc();

    translate( [-dxmin/2 + 11/2, 0, qz] ) rotate( [0,-90,0] ) feetech_scs20_servo( extrude_21mm_axles=0 );
    translate( [-dxmin/2 - 0.8, 0, -32] ) rotate( [0,90,0] ) feetech_servo_disc();

  }
} // end finger_v2



/* ***************************************************** */
/* ***************************************************** */
/* two_finger_gripper */
/* ***************************************************** */
/* ***************************************************** */


module two_finger_gripper()
{
  hh =  11;
  dx =  0;
  dy =  40;
  
  echo( "two_finger_gripper..." );
  if (make_two_finger_gripper_base) {
    // pa10_servo_hand_adapter( hh=hh );
    sunrise_M3207_servo_hand_adapter( hh=hh );
  }
  if (make_two_finger_gripper_fingers) {
    translate( [dx,-dy,49] ) finger_v2();
    translate( [dx,+dy,49] ) finger_v2();
  }
}



module three_finger_gripper()
{
  hh =  11;
  dx =  28.5;
  dy =  40;
  
  echo( "three_finger_gripper..." );
   {
    // pa10_servo_hand_adapter( hh=hh );
    sunrise_M3207_servo_hand_adapter( hh=hh );
  }

  // proximal servo
  translate( [10,-40,0] ) rotate( [90,0,180] ) 
    feetech_scs20_servo( extrude_mounting_screws=0 );


  {
    translate( [ 10,-40,59] ) finger_v2();
    translate( [-dx,+dy,49] ) finger_v2();
    translate( [+dx,+dy,49] ) rotate( [0,0,180] ) finger_v2();
  }
}



/**
 * a basic two-joint -finger using two SCS fingers,
 * with a small gap between the proximal and distal servos.
 * The body of the distal servo is used as the grasping fingertip,
 * but optional components could be mounted to the distal servo.
 * This finger is aligned with the axis along z-axis, with the
 * fingertip pointing along the x-axis and flexing inwards to +y.
 *
 * Origin is at the axis of the proximal servo.
 */
module basic_two_servo_finger()
{
  // echo( "basic_two_servo_finger..." );

  // proximal servo
  translate( [0,0,0] ) 
    feetech_scs20_servo( extrude_mounting_screws=0 );
  translate( [0,0,6.3] ) rotate( [0,180,0] ) 
    feetech_servo_disc(); 

  // distal servo  
  translate( [48,0,0] ) 
    feetech_scs15_servo( extrude_mounting_screws=0 );
  translate( [48,0,6.3] ) rotate( [0,180,0] ) 
    feetech_servo_disc(); 
  
//  translate( [48,0,-42] ) rotate( [0,180,0] ) 
//    feetech_servo_double_end_disc(); 

  // proximal-distal connector
  x1 = 22; y1=20-eps; z1=45;
  
  zb = -46+3; d_bottom_axle = 6.3; // or 10.0
  
  color( "lightblue" )
  difference() {
    union() {
      translate( [30+2-x1/2, 0, -20] )
        cube( [x1,y1,z1], center=true );
      translate( [30+2,0,3] )
        cube( [33,20,1.5], center=true );
      translate( [48,0,3] )
        difference() {
         cylinder( d=20.0, h=1.5, center=true, $fn=fn );
         cylinder( d=12.0, h=1.6, center=true, $fn=fn );
        }
      
      // bottom stuff: lever for mounting-disc   
      hhb = 2.0; // lever height
      difference() {
        union() {
          difference() {
            translate( [30+2,0,zb] )
              cube( [33,20,hhb], center=true );
            translate( [48,0,zb] )
              cylinder( d=d_bottom_axle, h=hhb+1, center=true, $fn=fn );
          }         
          translate( [48,0,zb] ) {
            difference() {
              cylinder( d=20.0, h=hhb, center=true, $fn=fn );
              cylinder( d=d_bottom_axle, h=hhb+1, center=true, $fn=fn );
            }
          }
        }
        translate( [48,0,-46] ) {
          for( theta=[0,90,180,270] )
            rotate( [0,0,theta] ) 
              translate( [7.2,0,0] )
                cylinder( d=2.5, h=hhb+1, center=true, $fn=fn );
        }
      }
    }

    // now subtract the servos with extrusions to get the mounting bores
    translate( [0,0,0.3] ) scale( [1,1.001,1.02] ) 
      feetech_scs20_servo( extrude_mounting_screws=8, d_mounting_screws=2 );
    translate( [48, 0,0.3] ) scale( [1,1.001,1.02] ) 
      feetech_scs15_servo( extrude_mounting_screws=8, d_mounting_screws=2 );
  }
}




/**
 * centered, oriented along +x, cut for x<0
 */
module cylindrical_fingertip(
  xsize = 20,
  ysize = 20,
  zsize = 40-eps,
)
{
  intersection() {
    scale( [xsize, ysize, zsize] )
      cylinder( d=1, h=1, center=true, $fn=fn );
    translate( [xsize/2,0,0] ) 
      cube( [xsize,ysize,zsize], center=true );
  }   
}


module elliptical_fingertip(
  xsize = 20,
  ysize = 20,
  zsize = 40
)
{
  intersection() {
    scale( [2*xsize,1.2*ysize,1.5*zsize] ) 
      sphere( d=1, center=true, $fn=fn );
    translate( [xsize/2,0,0] ) 
      cube( [xsize,ysize,zsize], center=true );
  } 
}




module flat_tactile_matrix(
  x_size = 40.0, 
  y_size = 40.0,
  overall_thickness = 3.0,
  n_cells = [6,6],
  cell_size= 5.5,
  cell_spacing = 1.0,
  base_color = "silver",
  cell_color = "darkgreen",
)
{
  if (overall_thickness < 1.0) {
    echo( "flat_tactile_matrix: overall_thickness < 1.0:", overall_thickness );
  }
  
  color( base_color )  
  translate( [0, 0, (overall_thickness-1)/2] )
    cube( [x_size, y_size, overall_thickness-1], center=true );
    
  // n_cells even: ... -1.5 -0.5 0.5 1.5 ...  odd:  -2 -1 0 1 2 ...
  // yy_offset = ((n_cells[1] % 2) == 1) ? -0.5 : 0.0;

  xx_offset = ((n_cells[0] % 2) == 1) 
              ?  -(n_cells[0]/2) * (cell_size + cell_spacing)
              :  -(n_cells[0]/2-0.5) * (cell_size + cell_spacing);

  yy_offset = ((n_cells[1] % 2) == 1) 
              ?  -(n_cells[1]/2) * (cell_size + cell_spacing)
              :  -(n_cells[1]/2-0.5) * (cell_size + cell_spacing);

//  echo( xx_offset, yy_offset );
//  echo( 7/2, 8/2, 7/2.001 );

  dz = overall_thickness-1+0.5;
  for( ix = [0 : n_cells[0]-1] ) {
    for( iy = [0 : n_cells[1]-1] ) {
      // echo( ix, iy );
      translate( [xx_offset + ix*(cell_size + cell_spacing),
                  yy_offset + iy*(cell_size + cell_spacing),
                  dz] )
      color( cell_color )
        cube( [cell_size, cell_size, 1], center=true );
    }
  }  
} // flat_tactile_matrix










module screw_M6_10_countersunk() {
  union() {
    cylinder( d=2*6, h=1, center=false, $fn=20 );
    translate( [0,0,1] ) cylinder( r1=6, r2=3, h=3, center=false, $fn=20 );
    translate( [0,0,3] )   cylinder( d=6, h=10, center=false, $fn=20 );
  }
}

module screw_M6_10_countersunk_extra_rim( length=10, extra_rim=3 ) {
  r = 3.1; rim = 1; eps=0.1; dr = 3; // head radius minus screw radius
  // standard rim length seems to be 1 mm on M6
  union() {
    cylinder(  r=r+dr, h=rim+extra_rim+2*eps, center=false, $fn=20 );
    translate( [0,0,rim+extra_rim+eps] )     cylinder( r1=r+dr, r2=r, h=3, center=false, $fn=20 );
    translate( [0,0,rim+extra_rim+dr] )       cylinder( r=r, h=length, center=false, $fn=20 );
  }
}


module screw_M6_15_countersunk() {
  union() {
    cylinder( d=2*6, h=1, center=false, $fn=20 );
    translate( [0,0,1] ) cylinder( r1=6, r2=3, h=3, center=false, $fn=20 );
    translate( [0,0,1] )   cylinder( d=6, h=15, center=false, $fn=20 );
  }
}

module screw_M4( length=10, head_length=3, head_diameter=7,
                 washer_thickness=0, washer_diameter=10 ) {
  r = 2.1; eps=0.1;
  union() {
    cylinder( d=head_diameter, h=head_length+eps, center=false, $fn=20 ); 
    translate( [0,0,head_length] ) cylinder( r=r, h=length, center=false, $fn=20 );
    translate( [0,0,head_length] ) cylinder( d=washer_diameter, h=washer_thickness, center=false, $fn=20 );
  }
}


module nut_M4( diameter=8.1, height=2.5 ) {
  cylinder( d=diameter, h=height, $fn=6, center=false );
}

module nut_M5( diameter=9.0, height=4.0 ) {
  cylinder( d=diameter, h=height, $fn=6, center=false );
}

module screw_M5( length=10, head_length=3, head_diameter=8.5,
                 washer_thickness=1, washer_diameter=11.0 ) {
  r = 2.6; eps=0.1;
  union() {
    cylinder( d=head_diameter, h=head_length+eps, center=false, $fn=20 ); 
    translate( [0,0,head_length] ) cylinder( r=r, h=length, center=false, $fn=20 );
    translate( [0,0,head_length] ) cylinder( d=washer_diameter, h=washer_thickness, center=false, $fn=20 );
  }
}


/**
 * basic distance ring, origin is at the center 
 */
module distance_ring(
  outer = 8.0, inner = 6.0, height = 5.0,
)
{
  difference() {
    cylinder( d=outer, h=height, $fn=fn, center=true );
    cylinder( d=inner, h=height+eps, $fn=fn, center=true );
  }  
}




// adapter plate to mount the servo hand directly onto a MHI PA10-6C
// robot, using four drill-holes for M6 Senkkopfschrauben.
// The plate has a circular set of 8 z-axis M4 nut drill holes 
// to connect along z-axis to other plates, and 4 radial
// M4 nut drill holes for mounting tools along x- and y-axis.
// 
module pa10_servo_hand_adapter( 
  hh=11.0,     // total plate thickness
) 
{
  eps = 0.1;
  
  dd = 98.0;   // outer diameter of PA-10 flange 
  d1 = 63.0;   // diameter for PA-10 M6 mount screws
  d2 = 40.0;   // diameter of inner wire-through hole
 
  h2 =  4.0;   // M6 screws Senkkopf height
  h3 =  4.0;   // M4 screws head plus washer height
 
  M4 = 4.2;
  M5 = 5.2;
  M6 = 6.2;

  screw_inner_diameter = M6;
  screw_head_diameter = 2*M6; 
  m4_washer_diameter = 10.0;
  
  n_outer_drill_holes = 8;
  r_outer_drill_holes = 43.0;

  n_pa10_drill_holes = 4;
  r_pa10_drill_holes = 63.0/2;
  
  //color( [0.8,0.7,0.6] )
  difference() {
    // actual adapter plate
    cylinder( d=dd, h=hh, center=false, $fn=100 );
    
    
    // arm end plate
    // %translate( [0, 0, -hh-2]) cylinder(d=76, h=10, center=false, $fn=100);
    
    // inner boring for tools and wires
    translate( [0, 0, -eps] )
      cylinder( d=d2, h=hh+2*eps, center=false, $fn=100 );    

    // PA-10 countersunk screws, four M6 drill holes
    dz = 3;
    for( i=[0:1:(n_pa10_drill_holes-1)] ) {      
      rotate( [0, 0, i*360.0/n_pa10_drill_holes] ) 
        rotate( [0, 180, 0] ) translate( [d1/2, 0, -hh-dz+1 ] ) 
          screw_M6_10_countersunk_extra_rim( length=(6+5*eps), extra_rim=dz );
    }
    
    // outer screws, eight M4 full-through drill holes
    for( i=[0:1:(n_outer_drill_holes-1)] ) {      
      rotate( [0, 0, 22.5+i*360.0/n_outer_drill_holes] ) 
        translate( [r_outer_drill_holes, 0, -1.1*eps ] ) 
          cylinder( h=hh+2.1*eps, d=M5, center=false );
    }

    // outer screws, eight drill holes for M4 washers and nuts
    for( i=[0:1:(n_outer_drill_holes-1)] ) {      
      rotate( [0, 0, 22.5+i*360.0/n_outer_drill_holes] ) 
        translate( [r_outer_drill_holes, 0, -3*eps ] ) 
          rotate( [0,0,30] ) 
          // cylinder( h=h3, d=m4_washer_diameter, center=false, $fn=6 );
          nut_M5(); 
    }
  }
}


module sunrise_M3207_servo_hand_adapter(
  hh = 10.0, // total plate thickness
) 
{
  dd = 74.0; // M3207 outer diameter
  d2 = 40.0; // diameter of fine-screw for drill chunk (previous value of 8.5 was WRONG!)
  
  xx = 56.0;
  yy = 100.0;
  
  fy = 40;
  
  n_m3207_drill_holes = 4;
  r_m3207_drill_holes = 28.28;
  
  M3INBUSs = 5.5; // inbus head diameter
  M3INBUSm = 3.0; // inbus head height

  
  rotate( [0,180,0] )
  difference() {

    // main body: ring for screwing onto M3207 sensor plus finger carriers
    union() {
      cylinder( d=dd, h=hh, center=false, $fn=100 ); 
      translate( [0,0,hh/2] )
        cube( [xx, yy, hh], center=true );
    }
    
    // central bore (cable passthrough)
    translate( [0, 0, -1.5*eps] ) cylinder( d=d2, h=hh+2*eps, center=false, $fn=50 );     
    
    // M3207 screws, four M4 drill holes
    for( i=[0:1:(n_m3207_drill_holes-1)] ) {      
      rotate( [0, 0, i*360.0/n_m3207_drill_holes+45] ) 
        translate( [r_m3207_drill_holes, 0, -eps ] ) 
          screw_M4( length= 10 );
    }
    
    hhh = 6; // load-bearing (flesh) depth of M3 routing channel
    // finger mounting screws: 2x5 M3
    for( dy=[-fy,+fy] ) {
      for( dx=[-20,-10,0,10,20] ) {
        translate( [dx, dy, -eps] )
          cylinder( d=3.2, h=hh+2*eps, $fn=fn, center=false );
        translate( [dx, dy, hh-hhh] )
          cylinder( d=M3INBUSs+0.3, h=hhh+2*eps, $fn=fn, center=false );
      }
    }
    
  }
}

