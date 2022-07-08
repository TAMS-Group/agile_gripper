w = 55;
h = 30;
overhang = 2;
d = 20 + overhang;
base_height = 8;
base_thickness = 3;
slant = 8;
top_thickness = 3;
eps = 0.1;
$fn=20;
difference(){
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
    translate([-w/2+slant, -d+4, base_thickness])cube([w-2*slant,d-3, h-base_thickness-top_thickness], center=false);
translate([-50, 0, h])rotate([-90, 0, -90])linear_extrude(100)polygon([
            [-eps, -eps],
            [d-overhang, -eps],
            [-eps, h-base_height],
        ]);
    for(i=[0:3]) {
    translate([-15 + i * 10, -5, -eps]) {
        cylinder(h=40, r=1.5);
        translate([0, -10, 0])cylinder(h=40, r=1.5);
    }}
}

translate([0, -10, 0])rotate([180, 0, 0])feetech_scs_bracket_big();
use <feetech_servo.scad>