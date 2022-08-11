w = 55;
h = 38;
overhang = 4.5;
d = 20 + overhang;
backhang = 20;
base_height = 8;
base_thickness = 3;
slant = 12;
top_thickness = 3;
eps = 0.1;
$fn=80;

use <digit_sensor.scad>
//hull()

// translate([-40, 5, 24])rotate([0, 90, 0])cylinder(h=80, d=3);

difference(){
    union() {
        rotate([90, 0, 0])linear_extrude(d) {
            polygon([
                [-w / 2, 0],
                [w/2, 0],
                [w/2, base_height],
                [(w/2)-slant, h],
                [-(w/2)+slant, h],
                [-w / 2, base_height],
            ]);
        }
        for(i=[-floor(backhang/2.5)+1 : floor(d/2.5)]) {
            translate([-w/2+slant+1, -i*2.5, h])cube([w-2*slant-2, 1, 1]);
        }
        translate([-(w/2 - slant), 0, base_height])rotate([90, 0, 90])linear_extrude(w-2*slant){
    polygon([
        [0, 0],
        [0, h-base_height], 
        [backhang, h-base_height]
    ]);
}
    }
    translate([-40, 11, 30])rotate([0, 90, 0])cylinder(h=80, d=3);
    translate([-40, 6, 22.5])rotate([0, 90, 0])cylinder(h=80, d=3);
    minkowski()
    {   
        hull()translate([0, -10, 0])rotate([0, 0, -90])digit_sensor(usb_connector=[]);
        translate([-0.75, -5, -5])cube([1.5, 20, 8]);
    }
    translate([-20, -10, -0.1])cylinder(r=1.5, h=100);
    translate([20, -10, -0.1])cylinder(r=1.5, h=100);
    translate([-20, -10, 4])cylinder(r=4, h=100);
    translate([20, -10, 4])cylinder(r=4, h=100);
    
    
    translate([20, -90, 10])rotate([0, 90, 90])cylinder(h=80, d=4);
    translate([-20, -90, 10])rotate([0, 90, 90])cylinder(h=80, d=4);
    translate([13, -85, 34])rotate([0, 90, 90])cylinder(h=80, d=4);
    translate([-13, -85, 34])rotate([0, 90, 90])cylinder(h=80, d=4);
    //translate([-w/2+slant, -d+4, base_thickness])cube([w-2*slant,d-3, h-base_thickness-top_thickness], center=false);
//translate([-50, 0, h])rotate([-90, 0, -90])linear_extrude(100)polygon([
//            [-eps, -eps],
//            [d-overhang, -eps],
//            [-eps, h-base_height],
//        ]);
//    for(i=[0:3]) {
//    translate([-15 + i * 10, -5, -eps]) {
//        cylinder(h=40, r=1.5);
//        translate([0, -10, 0])cylinder(h=40, r=1.5);
//    }}
}

module drilled_bracket() {
    difference() {
        feetech_scs_bracket_big();
        translate([-20, 0, -0.01])cylinder(r=1.5, h=100);
        translate([20, 0, -0.01])cylinder(r=1.5, h=100);
        translate([-5, 0, -0.01])cylinder(r=1, h=100);
        translate([5, 0, -0.01])cylinder(r=1, h=100);
    }
    
    }
// translate([0, -10, 0])rotate([180, 0, 0]) drilled_bracket();
use <feetech_servo.scad>

// translate([0, -10, 0])rotate([0, 0, -90])digit_sensor();
