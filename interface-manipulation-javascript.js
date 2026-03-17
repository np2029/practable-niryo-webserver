// Instead of creating and hosting my own interface, 
// this javascript converts the default debug interface into one that more accurately matches the interface
// designed and investigated as part of my dissertation.
//
// It is intended to be used only with the 'pyWebServer.py' file also included in this repository.
//
//  to use this interface, simply copy and paste this entire code file into the console after loading the debug interface

// get and remove uncessesary elements
document.getElementById("stream-information").parentElement.remove();
document.getElementById("show-config-file").parentElement.remove();
document.getElementById("show-data").parentElement.remove();
document.getElementById("clear-button").remove();
document.getElementById("second-row").remove();

// resize the row we need
document.getElementById("first-row").style.height = "90dvh";

// get elements we actually need
var camerafeed = document.getElementById("webcam-stream");

// need to break down the command box into parts
var inputfield = document.getElementById("input-command");
var sendButton = document.getElementById("send-button");

// change the widths and heights
camerafeed.parentElement.style.width = "60dvw";
sendButton.parentElement.parentElement.parentElement.style.width = "35dvw";
document.getElementById("send-data").style.height = "90dvh";

// make fields not draggable
sendButton.parentElement.parentElement.parentElement.setAttribute("draggable","false");
camerafeed.parentElement.setAttribute("draggable","false");

// make the actual command field and submit button invisible
inputfield.style.display = "none";
sendButton.style.display = "none";

// slider class lmao
class Slider{
    constructor(domElement, top, right, bottom, left, width, height, defaultPosition, min, max, labelText) {
        // assignments
        this.pos = defaultPosition;
        this.min = min;
        this.max = max;

        // give the domElement the slider class
        domElement.setAttribute("class","slider");

        // positioning
        domElement.style.position = 'relative';
        domElement.style.top = ''+top+'px';
        domElement.style.right = ''+right+'px';
        domElement.style.bottom = ''+bottom+'px';
        domElement.style.left = ''+left+'px';

        domElement.style.width = width+"px";
        domElement.style.height = height+"px";

        // create the slider input
        this.sliderInput = document.createElement("input");
        this.sliderInput.setAttribute("ondragstart","return false");
        this.sliderInput.setAttribute("type","range");
        this.sliderInput.setAttribute("min",""+this.min);
        this.sliderInput.setAttribute("max",""+this.max);
        this.sliderInput.setAttribute("value",""+this.pos);
        this.sliderInput.setAttribute("step","0.1");
        
        this.sliderInput.style.width = ''+width+'px';
        this.sliderInput.style.height = ''+height+'px';
        this.sliderInput.style.transform = 'rotate('+(this.angle)+'deg)';
        this.sliderInput.style.zIndex = 1;

        domElement.appendChild(this.sliderInput)
        
        // create the textbox
        this.textBox = document.createElement("input");
        this.textBox.setAttribute("type", "text");
        this.textBox.setAttribute("value", ""+this.pos);
        this.textBox.style.height = "20px";
        this.textBox.style.width = "50px";
        this.textBox.style.position = "relative";
        this.textBox.style.bottom = (parseInt(height)+30)+"px";
        this.textBox.style.left = /*""+(width*0.5 -25)+"px";*/"0px";

        domElement.appendChild(this.textBox);

        // create the label
        // constant for easier editing
        let labelHeight = 18;

        this.label = document.createElement("p");
        this.label.innerText = labelText;
        this.label.style.width = ""+width+"px";
        this.label.style.height = ""+labelHeight+"px"; 
        this.label.style.textAlign = "center";
        this.label.style.position = "relative";
        this.label.style.bottom = ""+(labelHeight+parseInt(height)+60)+"px";
        this.label.style.fontFamily = "Calibri";
        this.label.style.fontSize = "18px";
        this.label.style.zIndex = 0;
        domElement.appendChild(this.label);

        // create left and right limit text
        // left
        this.leftLimitText = document.createElement("p");
        this.leftLimitText.innerText = ""+min;
        this.leftLimitText.style.width = "10px";
        this.leftLimitText.style.height = "10px";
        this.leftLimitText.style.textAlign = "center";
        this.leftLimitText.style.position = "relative";
        this.leftLimitText.style.fontFamily = "Calibri";
        this.leftLimitText.style.fontSize = "18px";
        this.leftLimitText.style.bottom = "100px";
        
        domElement.appendChild(this.leftLimitText);

        // right
        this.rightLimitText = document.createElement("p");
        this.rightLimitText.innerText = ""+max;
        this.rightLimitText.style.width = "10px";
        this.rightLimitText.style.height = "10px";
        this.rightLimitText.style.textAlign = "center";
        this.rightLimitText.style.position = "relative";
        this.rightLimitText.style.fontFamily = "Calibri";
        this.rightLimitText.style.fontSize = "18px";
        this.rightLimitText.style.bottom = "130px";
        this.rightLimitText.style.left = (width - (Math.floor(Math.log10(width)+1)*5))+"px";// overly engineered, will adjust label based on digit count

        domElement.appendChild(this.rightLimitText);

        // add event listeners for slider movement
        this.sliderInput.oninput = (ev) => {
            this.textBox.value = this.sliderInput.value;
        }

        this.textBox.addEventListener("input", (ev) => {
            this.sliderInput.value = this.textBox.value;
        })

        this.textBox.addEventListener("focusout", (ev) => {
            // clamp to min and max
            this.textBox.value = Math.max(this.textBox.value, this.min);
            this.textBox.value = Math.min(this.textBox.value, this.max);

            // check for NaN
            if (isNaN(this.textBox.value))
                this.textBox.value = 0;

            this.sliderInput.value = this.textBox.value;
        })
    }
}// end of slider class

// array storing maximum angle data. Stored here for convenience
// format is: [minimum angle, maximum angle]
// NOTE: this is in degrees and is NOT to be used as arguments to set the arm position
const jointLimits = [
    [-169, 169], // BASE
    [-105, 35], // SHOULDER
    [-77, 90], // ELBOW
    [-120, 120], // WRIST PITCH
    [-110, 110], // WRIST ROLL
    [-145, 143], // WRIST YAW
]

// NOTE: this is in degrees and is NOT to be used as arguments to set the arm position
const homePosition = [0,28.6,-71.6,0,0,0] // CORRECT LATER

// create slider divs
var slider0Div = document.createElement("div");
var slider1Div = document.createElement("div");
var slider2Div = document.createElement("div");
var slider3Div = document.createElement("div");
var slider4Div = document.createElement("div");
var slider5Div = document.createElement("div");

slider0Div.id = "slider0";
slider1Div.id = "slider1";
slider2Div.id = "slider2";
slider3Div.id = "slider3";
slider4Div.id = "slider4";
slider5Div.id = "slider5";

sendButton.parentElement.parentElement.appendChild(slider0Div);
sendButton.parentElement.parentElement.appendChild(slider1Div);
sendButton.parentElement.parentElement.appendChild(slider2Div);
sendButton.parentElement.parentElement.appendChild(slider3Div);
sendButton.parentElement.parentElement.appendChild(slider4Div);
sendButton.parentElement.parentElement.appendChild(slider5Div);

const sliderSpacing = 30;
// create real sliders
const slider0 = new Slider(slider0Div,0,0,0,0,470,80,homePosition[0],jointLimits[0][0],jointLimits[0][1],"Base");
const slider1 = new Slider(slider1Div,1*sliderSpacing,0,0,0,470,80,homePosition[1],jointLimits[1][0],jointLimits[1][1],"Shoulder");
const slider2 = new Slider(slider2Div,2*sliderSpacing,0,0,0,470,80,homePosition[2],jointLimits[2][0],jointLimits[2][1],"Elbow");
const slider3 = new Slider(slider3Div,3*sliderSpacing,0,0,0,470,80,homePosition[3],jointLimits[3][0],jointLimits[3][1],"Forearm Roll");
const slider4 = new Slider(slider4Div,4*sliderSpacing,0,0,0,470,80,homePosition[4],jointLimits[4][0],jointLimits[4][1],"Wrist Pitch");
const slider5 = new Slider(slider5Div,5*sliderSpacing,0,0,0,470,80,homePosition[5],jointLimits[5][0],jointLimits[5][1],"Wrist Yaw");


// create and place submit button
var realSubmitButton = document.createElement("button");
realSubmitButton.innerText = "Send Move";
sendButton.parentElement.parentElement.appendChild(realSubmitButton);

realSubmitButton.style.position = "relative";
realSubmitButton.style.top = "150px";
realSubmitButton.style.height = "50px";

// need a function to make a move. for now, just use moveJoints
function submitMove() {
    // create move command
    // need to convert to radians
    com = "{"+
    "\"command\":\"moveJoints\","
    +"\"j0\":\"" + "\"" + slider0.sliderInput.value +"\","
    +"\"j1\":\"" + "\"" + slider1.sliderInput.value +"\","
    +"\"j2\":\"" + "\"" + slider2.sliderInput.value +"\","
    +"\"j3\":\"" + "\"" + slider3.sliderInput.value +"\","
    +"\"j4\":\"" + "\"" + slider4.sliderInput.value +"\","
    +"\"j5\":\"" + "\"" + slider5.sliderInput.value +"\""
    +"}";

    // set form command input text to new command
    inputfield.value = com;

    // execute command
    sendButton.click();
}

// shortcut, I want this on a button
// this can just use the "goHome" command, but that wont update the sliders propperly. have to do that manually
function goHome() {
    // hard code slider positions and inputs
    slider0.sliderInput.value = homePosition[0];
    slider0.textBox.value = homePosition[0];

    slider1.sliderInput.value = homePosition[1];
    slider1.textBox.value = homePosition[1];

    slider2.sliderInput.value = homePosition[2];
    slider2.textBox.value = homePosition[2];

    slider3.sliderInput.value = homePosition[3];
    slider3.textBox.value = homePosition[3];

    slider4.sliderInput.value = homePosition[4];
    slider4.textBox.value = homePosition[4];

    slider5.sliderInput.value = homePosition[5];
    slider5.textBox.value = homePosition[5];

    // set form command input text to goHome command
    inputfield.value = '{"command":"goHome"}';

    // execute command
    sendButton.click();
}

// link submit button to real submit action
realSubmitButton.addEventListener("click", submitMove)

// MAKE THE WINDOW FULLSCREEN!!! CAN'T DO THIS AUTOMATICALLY, JUST PRESS F11