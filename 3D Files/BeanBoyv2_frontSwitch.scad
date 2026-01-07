$fn = $preview?20:80;

pcbLen = 35.5;
pcbWid = 61.3;
pcbHigh = 1.5;
totalHigh = 5.5;

beanDoorHeight = 2;
beanDoorTolerance = .5;

beanRound = 4.5;
beanLength = pcbLen + 2.2*beanRound;
beanWidth = pcbWid +2.2*beanRound;
beanHeight = 23+beanDoorHeight;

lHoleHeight = 1;
lHoleIR = 4.5;
lHoleOR = 8;

translate([beanLength, 0, beanHeight])
rotate([0,180,0]) 
//Make the bean shape
difference() {

    //Main body
    union(){
        BeanBox();

        //Add a lanyard hole
        translate([beanLength/2,beanWidth+beanRound*0.4,beanHeight/2-lHoleHeight/2])
        
        minkowski() {
            linear_extrude(height = lHoleHeight) 
            difference() {
                offset(delta = 1) 
                LanyardHoleShape();

                LanyardHoleShape();
            } 

            sphere(r=2);
        }
        
    }
    
    
    //Cut out the PCB from the top front of the bean
    translate([(beanLength-pcbLen)/2, (beanWidth-pcbWid)/2, beanHeight-totalHigh])
    pcb();

    //Standard cube, roughly a standin for the 9V battery 
    translate([(beanLength-pcbLen)/2, (beanWidth-pcbWid)/2, 0])
    cube([pcbLen,pcbWid,beanHeight-totalHigh]);
    
    //Slidey door
    translate([(1.1*beanRound+pcbLen)+beanDoorTolerance/2,beanWidth,0])
    rotate([0,0,180])
    slideyLid(tolerance = beanDoorTolerance); 
}




translate([-45,0,0])
intersection() {
    slideyLid(); 
    translate([-beanRound,0,0]) 
    BeanBox();
}

translate([52,0,0])
SwitchFlipper(); 

module LanyardHoleShape(){
    hull(){
        for(i=[-4,4]){
            translate([i,0,0])
            circle(r=4); 
        }
    }
}

module BeanBox(){
    hull(){
        for(x=[beanRound,beanLength-beanRound],y=[beanRound,beanWidth-beanRound],z=[beanRound,beanHeight-beanRound]){
            translate([x,y,z]) 
            sphere(r=beanRound);
        }   
    }
}

module pcb(){
    //[35,61,1.5]
    //Screen starts 22.2mm from bottom of pcb, screen+pcb is 4.5mm
    cube([pcbLen,pcbWid,pcbHigh]);

    //Screen
    slopeWid = 2;
    translate([0,pcbWid-38.3,0])
    union() {
        //The full screen, kept beneath a bezel
        cube([pcbLen,38.3,totalHigh-1]);
        //Visible portion of the screen, tall enough to poke out
        translate([2,2,0]){
            cube([pcbLen-4,31,totalHigh]);

            translate([0,0,totalHigh-slopeWid])
            hull(){
                cube([pcbLen-4,31,0.01]);

                translate([-slopeWid/2,-slopeWid/2,slopeWid-0.01])
                cube([pcbLen-4+slopeWid,31+slopeWid,0.01]);
            } 
        }
    }

    //Add a cutout for the ribbon cable, 7mm in from each side, 1.5mm in the y, full height of the screen
    translate([7,pcbWid,pcbHigh])
    cube([pcbLen-14, 1.5, totalHigh-pcbHigh-1]);
    

    //Hole for switch
    holeWidth = 8;
    translate([pcbLen-9.5,2.1,0])
    union(){
        cube([holeWidth,3.3,15]); 
        /*
        stretch = 1.5;
        translate([holeWidth/2,1.6,15])
        scale([holeWidth*stretch*1.1,holeWidth*stretch,24])
        sphere(d=1); 
        */
    }
    

    //Buttons
    translate([0.5,0.5,0]) 
    for(shift=[[7.3,14],[17.5,7],[27.1,14]]){
        translate([shift[0],shift[1],1.5]){
            cylinder(d=10,h=1.5);
            cylinder(d=6.5,h=6); 
        }
        
    }
}

module slideyLid(tolerance=0){
    hull(){
        cube([pcbLen+tolerance,pcbWid+beanRound*1.1,0.01]);

        translate([-beanDoorHeight, 0, beanDoorHeight-0.01])
        cube([pcbLen+2*beanDoorHeight+tolerance, pcbWid+beanRound*1.1, 0.01]); 
    }

    //Small notch on top
    notchHeight = 3;
    translate([0, 0,beanDoorHeight])
    cube([pcbLen+tolerance, beanRound*1.1, notchHeight]);


    translate([-beanDoorHeight+1.5, 6, 0])
    cylinder(r=1.5, h=beanDoorHeight); 
}

module SwitchFlipper(){
    //holewidth is [8,3] from pcb module
    //Switch is [1.5,0.5]
    //Throw leaves ~2mm on the short side on both sides when switch is flipped
    difference() {
        cube([5.6,3,beanRound]);

        translate([1.9,1.0,0])
        cube([1.8,1,1.6]); 
    }
}