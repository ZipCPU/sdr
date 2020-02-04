include <zipcpu.scad>
include <zbreakerlogo.scad>
include <GT.scad>

module zbreaker() {
	// Top left is power, right side is the FMC connector
	raw_length = 36.95;
	raw_width  = 72.89;
	raw_base_width = 49.93;
	//
	raw_subboard_length = 20.55;
	//
	// The extra 2.4 is to get above the tops of the wire connectors
	raw_height_above_board = 4.6 + 1.5 + 2.4;
	zh = 2;
	tol = 0.1;
	
	case_width  = 85;	// Must be greater than 5.5*2 + raw_width
	case_length = 50;
	subbase_height = 5;

	bx = (case_width-raw_width)/2;
	by = (case_length-raw_length)/2;

	rad = 2;
	lip=2;

	module cutout(x, y, w, h, v) {
		ex = 1.5;
		translate([x-ex/2,y-ex/2,-tol/2])
			cube([w+ex,h+ex,v+tol]);
	}

	module keepit(x, y, w, h, v) {
		ex = -1.5;
		translate([x-ex/2,y-ex/2,-tol/2])
			cube([w+ex,h+ex,v+tol]);
	}

	module	subbase() {
		ex = 0.8;
		lipsm = lip-ex/2;
		shelf=3;
		total_height = subbase_height + raw_height_above_board;
		difference() {
			// A big case shell to start with
			translate([0,0,-subbase_height])
				color("red")
				cube([case_width, case_length, total_height-0.1]);

			//
			// Carve out the inside
			//
			translate([lipsm,lipsm,-0.1])
				cube([case_width-2*lipsm, case_length-2*lipsm, raw_height_above_board+0.1]);

			//
			// Cutouts for the board itself
			translate([bx-ex/2,by-ex/2,-1.6])
				cube([raw_base_width+ex, raw_length+ex, 1.7+2*lipsm]);
			translate([bx-ex/2,by+(raw_length-raw_subboard_length)/2-ex/2,-1.6])
				cube([raw_width+ex,raw_subboard_length+ex,1.7+2*lipsm]);

			//
			// Cut outs for the solder points under the board
			translate([bx+4.88,by,-3.1])
				cube([38.74+ex, 8.01, 3.2]);
			translate([bx+4.88-ex/2,by+31.05,-3.1])
				cube([38.74+ex, raw_length-31.05, 3.2]);
			translate([bx-ex/2+40,
				by+(raw_length-raw_subboard_length)/2-ex/2+shelf,-3.1])
				cube([(raw_width-40)+ex-shelf,raw_subboard_length+ex-2*shelf,3.2]);

			// A cutout underneath the board, USB plug side
			translate([bx-ex/2, by-ex/2+shelf,-3.1])
				cube([10,raw_length+ex-2*shelf,3.2]);

			// Cutout underneath main board, opposite USB side
			translate([bx+raw_base_width-10, by-ex/2+shelf,-3.1])
				cube([10+ex/2,raw_length+ex-2*shelf,3.2]);


			//
			// Cut in, near the USB plug
			translate([0,0,-subbase_height-0.1])
				cutout(0,by+6.0,bx-2.5,19,subbase_height+0.1+2*lip);
			// USB plug cutout in the lip
			translate([0,0,0])
				cutout(0,by+6.0,bx+2*lipsm,19,subbase_height+0.1+2*lipsm);
			// PMOD cutout in the lip
			translate([0,0,0])
				cutout(0,by+6.0,bx+2*lipsm,19,total_height+0.1);
			translate([bx,by,0]) {
				cutout(5.5, 30.5,
					28.8+15.7-5.5,
					case_length-30.5,total_height+0.1);
			}


			//
			// Corner cutouts so we can round the corners (later)
			translate([-0.1,-0.1,-subbase_height-0.1])
				cube([rad+0.1,rad+0.1,total_height+0.2]);
			translate([case_width-rad,-0.1,-subbase_height-0.1])
				cube([rad+0.1,rad+0.1,total_height+0.2]);
			translate([-0.1,case_length-rad,-subbase_height-0.1])
				cube([rad+0.1,rad+0.1,total_height+0.2]);
			translate([case_width-rad,case_length-rad,-subbase_height-0.1])
				cube([rad+0.1,rad+0.1,total_height+0.2]);
		}

		difference() {
			// Round all the corners
			union() {
			// First, fill the corners with cylinders
			translate([rad,rad,-subbase_height])
				cylinder(h=total_height,r=rad);
			translate([case_width-rad,rad,-subbase_height])
				cylinder(h=total_height,r=rad);
			translate([case_width-rad,case_length-rad,-subbase_height])
				cylinder(h=total_height,r=rad);
			translate([rad,case_length-rad,-subbase_height])
				cylinder(h=total_height,r=rad);
			}

			// Then remove the inner parts of the cylinders where
			// they'd interfere with the board
			translate([lipsm,lipsm,-0.1])
				cube([case_width-2*lipsm, case_length-2*lipsm, raw_height_above_board+0.1]);
		}
	}

	module layer(n) {
		base = (n==0) ? 0
			: (n == 1) ? 1.6
			: (n == 2) ? 3.0
			: (n == 3) ? 4.7
			: raw_height_above_board;

		ztmp = (n==0) ? 1.6
			: (n == 1) ? 3.0
			: (n == 2) ? 4.7
			: raw_height_above_board;

		zh = ztmp - base;
		translate([bx,by,base]) {
			// RGB - Double pin header
			cutout(11.2,6.1-5.1,10.04,4.6,raw_height_above_board);
			// Flash - Single pin header
			cutout(41.60-17.6,3.35-2.5,17.60,2.0,raw_height_above_board);
			// Cut in, near the USB plug
			cutout(-bx,6.0,bx-2.5,19,zh);

			// 5V top (between PMods)
			//
			// 4.8 x 2.5
			cutout(23.81,31.28,2.5,5.5,raw_height_above_board+0.1);
			// 5V (right)
			cutout(44.25,6,5.5,2.5,raw_height_above_board+0.1);
			// CRESET header
			cutout(39,5.7,2.5,5.5,raw_height_above_board+0.1);

			// pad platform
			if (n == 0) {
				difference() {
					color("black") {
						translate([-0.4,-0.4,0])
						cube([raw_width+0.8,raw_length+0.8,zh]);
					}

					// The sub board
					keepit(59,0,66-59,15.7,zh);
					keepit(59,24,66-59,15.7,zh);

					// The tearoff point
					keepit(50-0.75+0.4,0,2+1.5-0.8,raw_length,zh);

					// Screws
					// cutout(0,0,6,6,zh);
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

	lipw = lip+0.4;
	if(1){
	rotate([180,0,0]) translate([0,-case_length,-raw_height_above_board])
	difference() {
		translate([lipw,lipw,0])
			cube([case_width-2*lipw,case_length-2*lipw,raw_height_above_board]);

		/*
		// Remove the corners
		translate([-0.1,-0.1,-0.1])
			cube([rad+0.1,rad+0.1,raw_height_above_board+0.2]);
		translate([case_width-rad,-0.1,-0.1])
			cube([rad+0.1,rad+0.1,raw_height_above_board+0.2]);
		translate([-0.1,case_length-rad,-0.1])
			cube([rad+0.1,rad+0.1,raw_height_above_board+0.2]);
		translate([case_width-rad,case_length-rad,-0.1])
			cube([rad+0.1,rad+0.1,raw_height_above_board+0.2]);
		*/

		// Detailed layers
		layer(0);
		layer(1);
		layer(2);
		layer(3);
		embossZipBreaker();
		embossGT();
		// embossBlackIce2();
		// For matching, remove the screw holes
		// screw_height = 4;
		// cutout(0,0,6.5,6.5,screw_height);
		// cutout(raw_width-6.5,0,6.5,6.5,screw_height);
		// cutout(raw_width-6.5,raw_length-6.5,6.5,6.5,screw_height);
		// cutout(0,raw_length-6.5,6.5,6.5,screw_height);
	}}

	if (1) { translate([0,case_length+10,subbase_height]) {
		difference() {
			subbase();
			translate([0,0,-2*subbase_height]) {
				embossZip();
			}
			embossGT2();
		}
	}}
}

zbreaker();
