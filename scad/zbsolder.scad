
include <zipcpu.scad>
include <zbreakerlogo.scad>
include <GT.scad>

module zbsolder() {
	// Top left is power, right side is the FMC connector
	raw_length = 36.95;
	raw_width  = 72.89;
	raw_base_width = 49.93;
	raw_thickness = 1.55;
	//
	raw_subboard_length = 20.55;
	//
	// The extra 2.3 is to get above the tops of the wire connectors
	raw_height_above_board = 4.6 + 1.5 + 2.3;
	zh = 2;
	tol = 0.1;
	
	case_width  = 85;	// Must be greater than 5.5*2 + raw_width
	case_length = 50;
	subbase_height = 3;

	bx = (case_width-raw_width)/2;
	by = (case_length-raw_length)/2;

	rad = 2;

	module cutout(x, y, w, h, v) {
		ex = 0.8;
		translate([x-ex/2,y-ex/2,-tol/2])
			cube([w+ex,h+ex,v+tol]);
	}

	module keepit(x, y, w, h, v) {
		ex = 0.8;
		translate([x+ex/2,y+ex/2,-tol/2])
			cube([w-ex,h-ex,v+tol]);
	}

	module pincutout(x, y, w, h, v) {
		ex = 0.8;
		hbase=2.5;
		if (1) {
			pinsz = 0.6;
			// Full size x,y initially, although much shorter
			translate([x-ex/2,y-ex/2,-ex/2])
				cube([w+ex,h+ex,hbase+ex]);

			// Now a restricted size
			if (w > h) {
				px = x; pw = w;
				py = y+h/2-pinsz/2; ph = pinsz;
				translate([px-ex/2,py-ex/2,-ex/2])
					cube([pw+ex,ph+ex,v+ex]);
			} else {
				px = x+w/2-pinsz/2; pw = pinsz;
				py = y; ph = h;
				translate([px-ex/2,py-ex/2,-ex/2])
					cube([pw+ex,ph+ex,v+ex]);
			}

		} else {
			cutout(x, y, w, h, v);
		}
	}

	module	rawcase() {
		ex = 0.8;
		shelf=3;
		total_height = raw_height_above_board + raw_thickness;
		difference() {
			cube([case_width, case_length, total_height]);

			//
			// Cutouts so we can round the corners (later)
			translate([-0.1,-0.1,-0.1])
				cube([rad+0.1,rad+0.1,total_height+0.2]);
			translate([case_width-rad,-0.1,-0.1])
				cube([rad+0.1,rad+0.1,total_height+0.2]);
			translate([-0.1,case_length-rad,-0.1])
				cube([rad+0.1,rad+0.1,total_height+0.2]);
			translate([case_width-rad,case_length-rad,-0.1])
				cube([rad+0.1,rad+0.1,total_height+0.2]);

			// Horizontal/width cuts, along X
			translate([-0.1,-0.1,total_height-rad])
				cube([case_width+0.2,rad+0.1,rad+0.2]);
			translate([-0.1,case_length-rad,total_height-rad])
				cube([case_width+0.2,rad+0.1,rad+0.2]);

			// Length cuts, along Y
			translate([-0.1,-0.1,total_height-rad])
				cube([rad+0.1,case_length+0.2,rad+0.2]);
			translate([case_width-rad,-0.1,total_height-rad])
				cube([rad+0.1,case_length+0.2,rad+0.2]);
		}
		union() {
			// Vertical cyclinders
			//
			// Z
			translate([rad,rad,0])
				cylinder(h=total_height-rad,r=rad);
			translate([case_width-rad,rad,0])
				cylinder(h=total_height-rad,r=rad);
			translate([case_width-rad,case_length-rad,0])
				cylinder(h=total_height-rad,r=rad);
			translate([rad,case_length-rad,0])
				cylinder(h=total_height-rad,r=rad);

			// X (width)
			translate([rad,rad,total_height-rad]) rotate([0,90,0])
				cylinder(h=case_width-2*rad,r=rad);
			translate([rad,case_length-rad,total_height-rad]) rotate([0,90,0])
				cylinder(h=case_width-2*rad,r=rad);

			// Y (length)
			translate([rad,rad,total_height-rad]) rotate([-90,0,0])
				cylinder(h=case_length-2*rad,r=rad);
			translate([case_width-rad,rad,total_height-rad]) rotate([-90,0,0])
				cylinder(h=case_length-2*rad,r=rad);

			// Corner sphere
			translate([rad,rad,total_height-rad])
				sphere(rad);
			translate([case_width-rad,rad,total_height-rad])
				sphere(rad);
			translate([case_width-rad,case_length-rad,total_height-rad])
				sphere(rad);
			translate([rad,case_length-rad,total_height-rad])
				sphere(rad);
		}
	}

	module	case() {
		ex = 0.8;
		shelf=3;
		total_height = raw_height_above_board + raw_thickness;
		difference() {
			translate([0,0,0])
				rawcase();

			// Cutouts for the board itself
			translate([bx-ex/2,by-ex/2, -0.1])
				cube([raw_base_width+ex, raw_length+ex,
					raw_thickness+0.1]);
			translate([bx-ex/2,
				by+(raw_length-raw_subboard_length)/2-ex/2,
				-0.1])
				cube([raw_width+ex,raw_subboard_length+ex,
					raw_thickness+0.1]);

			// Cut in, near the USB plug
			translate([0,0,-0.1])
				cutout(0,by+6.0,bx-2.5,19,raw_thickness+0.1);
			// USB plug cutout in the lip
			translate([0,-0.1,-0.1])
				cutout(0,by+6.0,bx+0.2,19,total_height+0.1);
			// PMOD cutout in the lip
			translate([-0.1,0,-0.1])
				cutout(0,by+6.0,bx+0.1,19,total_height+0.1);
			translate([bx,by,-0.1])
				cutout(5.5, 30.5,
					28.8+15.7-5.5,
					case_length-30.5,raw_thickness+0.1);


		}
	}

	module layer(n) {
		base = (n==0) ? -0.1
			: (n == 1) ? 1.6
			: (n == 2) ? 3.0
			: (n == 3) ? 4.7
			: raw_height_above_board;

		ztmp = (n==0) ? 1.6
			: (n == 1) ? 3.0
			: (n == 2) ? 4.7
			: raw_height_above_board;

		zh = ztmp - base;

		// RGBLED header
		// This is a 4x2 header, so we can't neck down on a
		// single row of pins like pincutout does
		cutout(11.2,6.1-5.1,10.04,4.6,raw_height_above_board+0.1);
		// Flash header
		pincutout(41.60-17.6,3.35-2.5,17.60,2.0,raw_height_above_board+0.1);
		// 5V top (between PMods)
		//
		// 4.8 x 2.5
		pincutout(23.81,31.28,2.5,4.8,raw_height_above_board+0.1);
		// 5V (right)
		pincutout(44.25,6,4.8,2.5,raw_height_above_board+0.1);
		// CRESET header
		pincutout(39,5.7,2.5,4.8,raw_height_above_board+0.1);

		translate([0,0,base]) {
			// Cut in, near the USB plug
			cutout(-bx,6.0,bx-2.5,19,zh);

			// pad platform
			if (n == 0) {
				difference() {

					color("black") {
						translate([-0.4,-0.4,0])
							cube([raw_base_width+0.8,raw_length+0.8,zh]);
						translate([-0.4,(raw_length-raw_subboard_length)/2,0])
							cube([raw_width+0.8,raw_subboard_length+0.8,zh]);
					}


					// The four corners (containing the
					// screw holes)
					keepit(-2,-2,11.6,8.5,zh);
					keepit(50-7.3,-2,9.3,6.3,zh);
					keepit(-2,37-9.2,6.6,11.2,zh);
					keepit(50-5,27.6,7,37-25.6,zh);

					//
					keepit(19.8,12.4,23.4-19.8,15.8-12.4,zh);
					keepit(19.6,22.5,26.25-19.6,29.7-22.5,zh);
					keepit(36,26.5,50-36,28.8-26.5,zh);

					// The sub board
					keepit(59,0,66-59,15.7,zh);
					keepit(59,24,66-59,15.7,zh);

					// The tearoff point
					keepit(50-0.75+0.4,0,2+1.5-0.8,raw_length,zh);


					// cutout(raw_width-6,0,6,6,zh);
					// cutout(raw_width-6,raw_length-6,6,6,zh);
					// cutout(0,raw_length-6,6,6,zh);

					// Pins on the left
					// keepit(-2,68.88-34.0,2.78+2,34.0,zh);
				}
			}

			if (n <= 1) {
				// @ 2.8mm, buttons on the end
				cutout(66.2,10.4,4.74,17.22,zh);
				// Button at the beginning
				cutout(-bx,7.8,bx+4.25,4.25,zh);
				// USB plug
				cutout(-bx,14.95,bx+5.1,7.7,zh);
			}
			//
			if (n <= 2) {
				// @ 4.6mm
				// Top Left PMod
				cutout(5.5,30.5,15.7,13.85,zh);
				// Top Right PMod
				cutout(28.8,30.5,15.7,13.85,zh);
				// Right edge PMod
				cutout(44,11,13.85,15.7,zh);

			}
		}
	}

	module	embossZip() {
		s = 1.0;
		depth = 1.5;
		translate([case_width/2, case_length/2, s*zlogoh/2+depth]) {
			rotate([0,180,180]) scale([s,s,1]) {
				translate([-zlogow/2,(zlogob-zlogol)/2,
							-zlogoh/2])
						zipcpulogo();
			}
		}
	}

	module	embossGT() {
		s = 0.8;
		margin=4;
		depth = 1.5;
		translate([case_width-gtlogo_w*s/2-margin, margin+gtlogo_l*s/2,
			   raw_height_above_board+gtlogo_h/2-depth]) {
			rotate([0,0,0]) scale([s,s,1]) {
				translate([-gtlogo_w/2,
						(-gtlogo_l)/2,
							-gtlogo_h/2])
						GTlogo();
			}
		}
	}

	module	embossGT2() {
		s = 0.8;
		margin=4;
		depth = 1.5;
		translate([case_width-gtlogo_w*s/2-margin, case_length-margin-gtlogo_l*s/2,
			   -subbase_height-gtlogo_h/2+depth]) {
			rotate([0,180,180]) scale([s,s,1]) {
				translate([-gtlogo_w/2,
						(-gtlogo_l)/2, -gtlogo_h/2])
					GTlogo();
			}
		}
	}

	module	embossZipBreaker() {
		s = 1.4;
		depth = 1.5;
		translate([case_width/2+2, case_length/2,
			   raw_height_above_board+zblogoh/2-depth]) {
			rotate([0,0,0]) scale([s,s,1]) {
				translate([-zblogow/2,
						(-zblogol)/2,
							-zblogoh/2])
						zbreakerlogo();
			}
		}
	}

	rotate([180,0,0]) translate([0,-case_length,-raw_height_above_board
				-raw_thickness])
	difference() {
		case();
		translate([bx,by,raw_thickness]) {

			// Detailed layers
			layer(0);
			layer(1);
			layer(2);
			layer(3);
			translate([-bx,-by,0]) {
				embossZipBreaker();
				embossGT();
			}
		}
	}
}

zbsolder();

// translate([0,100+10,8.4])
// rotate([180,0,0])
//	zbsolder();
