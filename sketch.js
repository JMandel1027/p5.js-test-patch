var theta, theta_prev;
var w0;

function setup(){
  createCanvas(windowWidth, windowHeight);
  xPos = windowWidth+1;
  stroke(255);
  strokeWeight(6);
  fill(70);

  size = windowHeight/12;
  textSize(size);
  textAlign(CENTER,CENTER);
}

function draw(){
  background(255);

  // w0 is the quaternion for the original orientation
  if(frameCount < 5){
   w0 = quaternion(rotationX,rotationY,rotationZ);
  }

  var w = quaternion(rotationX,rotationY,rotationZ);

  // we can extract roll, pitch, yaw from the quaternion
  var radtodeg = 180/PI;
  var roll  = radtodeg*Math.atan2(2*w[2]*w[0] - 2*w[1]*w[3], 1 - 2*w[2]*w[2] - 2*w[3]*w[3]);
  var pitch = radtodeg*Math.atan2(2*w[1]*w[0] - 2*w[2]*w[3], 1 - 2*w[1]*w[1] - 2*w[3]*w[3]);
  var yaw   = radtodeg*Math.asin(2*w[1]*w[2] + 2*w[3]*w[0]);

  text("roll: "+str(round(roll)),windowWidth/2,size);
  text("pitch: "+str(round(pitch)),windowWidth/2,2*size);
  text("yaw: "+str(round(yaw)),windowWidth/2,3*size);

  // quaternion values
  text("Qw: " + str(round(w[0]*100)/100),windowWidth/2,4*size);
  text("Qx: " + str(round(w[1]*100)/100),windowWidth/2,5*size);
  text("Qy: " + str(round(w[2]*100)/100),windowWidth/2,6*size);
  text("Qz: " + str(round(w[3]*100)/100),windowWidth/2,7*size);

  // tells us if the device is turned by 90 degrees
  deviceTurned(w,w0)

}

function deviceTurned(w,w0){
  var radtodeg = 180/PI;

  // quaternion distance = 1 - <q1,q2>^2 is a measure of difference between orientations
  // Goes between 0 (identical orientations) to 1 (opposite orientations)
  // quaternion angle = arccos(2*<q1,q2>^2 - 1) converts this into an angle
  // this is the angle of rotation needed to get from one orientation to another
  // i.e. the angle between two orientations
  var quatDistance = quaternionDistance(w,w0);
  var quatAngleBetween = radtodeg*quaternionAngleBetween(w,w0);

  text("quat dist: "+str(round(quatDistance*100)/100),windowWidth/2,8*size);
  text("quat angle: "+str(round(quatAngleBetween)),windowWidth/2,9*size);

  // define quaternion orientations for +/- 90 degree rotations in X, Y, Z
  var xcw = quaternion(90,0,0);
  var xccw = quaternion(-90,0,0);
  var ycw = quaternion(0,90,0);
  var yccw = quaternion(0,-90,0);
  var zcw = quaternion(0,0,90);
  var zccw = quaternion(0,0,-90);

  // quaternion distance = (1 - cos(quaternion angle))/2
  // so when the angle is 90 degrees, quaternion distance is 0.5
  if(quatDistance >= 0.5){
    text("Device TURNED",windowWidth/2,10*size);
    if(quaternionDistance(w,quaternionMultiply(w0,xcw))<0.1){text("X",windowWidth/2,11*size);}
    else if (quaternionDistance(w,quaternionMultiply(w0,xccw))<0.1){text("-X",windowWidth/2,11*size);}
    else if (quaternionDistance(w,quaternionMultiply(w0,ycw))<0.1){text("Y",windowWidth/2,11*size);}
    else if (quaternionDistance(w,quaternionMultiply(w0,yccw))<0.1){text("-Y",windowWidth/2,11*size);}
    else if (quaternionDistance(w,quaternionMultiply(w0,zcw))<0.1){text("Z",windowWidth/2,11*size);}
    else if (quaternionDistance(w,quaternionMultiply(w0,zccw))<0.1){text("-Z",windowWidth/2,11*size);}
  }
  /*
  if (quatAngleBetween <= 45) {
    background(255, 255, 255);
  } else if (quatAngleBetween <= 90) {
    background(255, 0, 0);
  } else if (quatAngleBetween <= 135) {
    background(0, 255, 0);
  } else if (quatAngleBetween <= 180) {
    background(0, 0, 255);
  }
  */
}

transposeRotationMatrix = function(mat){
   //check if matrix is square
  if(mat instanceof Array && mat.length === 9){
    var t00 = mat[0];
    var t10 = mat[1];
    var t20 = mat[2];
    var t01 = mat[3];
    var t11 = mat[4];
    var t21 = mat[5];
    var t02 = mat[6];
    var t12 = mat[7];
    var t22 = mat[8];

    return [t00, t01, t02,t10, t11, t12,t20, t21, t22];

  }
    else{
      throw new Error('Rotation Matrix should be an array of 9 elements');
    }
  };

rotationMatrix = function(angleX,angleY,angleZ) {

  // Following the implementation in
  // https://dev.opera.com/articles/w3c-device-orientation-usage/

  var degtorad = Math.PI / 180; // Degree-to-Radian conversion

  var _x = angleX  ? angleX  * degtorad : 0; // beta value
  var _y = angleY ? angleY * degtorad : 0; // gamma value
  var _z = angleZ ? angleZ * degtorad : 0; // alpha value
  var cX = Math.cos( _x );
  var cY = Math.cos( _y );
  var cZ = Math.cos( _z );
  var sX = Math.sin( _x );
  var sY = Math.sin( _y );
  var sZ = Math.sin( _z );

  // ZXY rotation matrix construction
  // (i.e. rotations are applied first Z, then X, then Y)
  // This is for consistency with deviceOrientation API
  // See http://w3c.github.io/deviceorientation/spec-source-orientation.html
  // Since the p5 3D renderer multiplies the rotation matrix from the left
  // and not the right, we're using transpose of the rotation matrix below

  var m00 = cZ * cY - sZ * sX * sY;
  var m01 = cY * sZ + cZ * sX * sY;
  var m02 = - cX * sY;
  var m10 = - cX * sZ;
  var m11 = cZ * cX;
  var m12 = sX;
  var m20 = cY * sZ * sX + cZ * sY;
  var m21 = sZ * sY - cZ * cY * sX;
  var m22 = cX * cY;

  return [m00, m01, m02, m10, m11, m12, m20, m21, m22];

};

rotateMatrix = function(rotmat){

  var a00 = this.mat4[0];
  var a01 = this.mat4[1];
  var a02 = this.mat4[2];
  var a03 = this.mat4[3];
  var a10 = this.mat4[4];
  var a11 = this.mat4[5];
  var a12 = this.mat4[6];
  var a13 = this.mat4[7];
  var a20 = this.mat4[8];
  var a21 = this.mat4[9];
  var a22 = this.mat4[10];
  var a23 = this.mat4[11];

  var b00,b01,b02,b10,b11,b12,b20,b21,b22;

  if (rotmat instanceof Array && rotmat.length === 9) {
    b00 = rotmat[0];
    b01 = rotmat[1];
    b02 = rotmat[2];
    b10 = rotmat[3];
    b11 = rotmat[4];
    b12 = rotmat[5];
    b20 = rotmat[6];
    b21 = rotmat[7];
    b22 = rotmat[8];
  }
  else{
    throw new Error('Rotation Matrix should be an array of 9 elements');
  }

  // rotation-specific matrix multiplication
  this.mat4[0] = a00 * b00 + a10 * b01 + a20 * b02;
  this.mat4[1] = a01 * b00 + a11 * b01 + a21 * b02;
  this.mat4[2] = a02 * b00 + a12 * b01 + a22 * b02;
  this.mat4[3] = a03 * b00 + a13 * b01 + a23 * b02;
  this.mat4[4] = a00 * b10 + a10 * b11 + a20 * b12;
  this.mat4[5] = a01 * b10 + a11 * b11 + a21 * b12;
  this.mat4[6] = a02 * b10 + a12 * b11 + a22 * b12;
  this.mat4[7] = a03 * b10 + a13 * b11 + a23 * b12;
  this.mat4[8] = a00 * b20 + a10 * b21 + a20 * b22;
  this.mat4[9] = a01 * b20 + a11 * b21 + a21 * b22;
  this.mat4[10] = a02 * b20 + a12 * b21 + a22 * b22;
  this.mat4[11] = a03 * b20 + a13 * b21 + a23 * b22;

  return this;
};

quaternion = function(angleX,angleY,angleZ) {

  // Following the implementation in
  // https://dev.opera.com/articles/w3c-device-orientation-usage/

  var degtorad = Math.PI / 180; // Degree-to-Radian conversion

  var _x = angleX  ? angleX  * degtorad : 0; // beta value
  var _y = angleY ? angleY * degtorad : 0; // gamma value
  var _z = angleZ ? angleZ * degtorad : 0; // alpha value

  var cX = Math.cos( _x/2 );
  var cY = Math.cos( _y/2 );
  var cZ = Math.cos( _z/2 );
  var sX = Math.sin( _x/2 );
  var sY = Math.sin( _y/2 );
  var sZ = Math.sin( _z/2 );

  // ZXY quaternion construction
  // (i.e. rotations are applied first Z, then X, then Y)
  // This is for consistency with deviceOrientation API
  // See http://w3c.github.io/deviceorientation/spec-source-orientation.html

  var w = cX * cY * cZ - sX * sY * sZ;
  var x = sX * cY * cZ - cX * sY * sZ;
  var y = cX * sY * cZ + sX * cY * sZ;
  var z = cX * cY * sZ + sX * sY * cZ;

  return [ w, x, y, z ];
};

rotateQuaternion = function(quat){

  var a00 = this.mat4[0];
  var a01 = this.mat4[1];
  var a02 = this.mat4[2];
  var a03 = this.mat4[3];
  var a10 = this.mat4[4];
  var a11 = this.mat4[5];
  var a12 = this.mat4[6];
  var a13 = this.mat4[7];
  var a20 = this.mat4[8];
  var a21 = this.mat4[9];
  var a22 = this.mat4[10];
  var a23 = this.mat4[11];

  var b00,b01,b02,b10,b11,b12,b20,b21,b22;

  if (quat instanceof Array && quat.length === 4) {

  // Note: We aren't checking if quaternion is normalized
  // Either implement this or make a note in the documentation
  var qw = quat[0];
  var qx = quat[1];
  var qy = quat[2];
  var qz = quat[3];

  // Since the p5 3D renderer multiplies the rotation matrix from the left
  // and not the right, we're using transpose of the rotation matrix below
  b00 = 1 - 2*qy*qy - 2*qz*qz;
  b01 = 2*qx*qy + 2*qz*qw;
  b02 = 2*qx*qz - 2*qy*qw;
  b10 = 2*qx*qy - 2*qz*qw;
  b11 = 1 - 2*qx*qx - 2*qz*qz;
  b12 = 2*qy*qz + 2*qx*qw;
  b20 = 2*qx*qz + 2*qy*qw;
  b21 = 2*qy*qz - 2*qx*qw;
  b22 = 1 - 2*qx*qx - 2*qy*qy;
}
  else{
    throw new Error('Quaternion should be an array of 4 elements');
  }

  // rotation-specific matrix multiplication
  this.mat4[0] = a00 * b00 + a10 * b01 + a20 * b02;
  this.mat4[1] = a01 * b00 + a11 * b01 + a21 * b02;
  this.mat4[2] = a02 * b00 + a12 * b01 + a22 * b02;
  this.mat4[3] = a03 * b00 + a13 * b01 + a23 * b02;
  this.mat4[4] = a00 * b10 + a10 * b11 + a20 * b12;
  this.mat4[5] = a01 * b10 + a11 * b11 + a21 * b12;
  this.mat4[6] = a02 * b10 + a12 * b11 + a22 * b12;
  this.mat4[7] = a03 * b10 + a13 * b11 + a23 * b12;
  this.mat4[8] = a00 * b20 + a10 * b21 + a20 * b22;
  this.mat4[9] = a01 * b20 + a11 * b21 + a21 * b22;
  this.mat4[10] = a02 * b20 + a12 * b21 + a22 * b22;
  this.mat4[11] = a03 * b20 + a13 * b21 + a23 * b22;

  return this;
};

 /* [rotateMatrix description]
  * @param  {Array} rotation matrix (returned from rotationMatrix function)
  * @return {p5.Renderer3D}      [description]
  */
Renderer3D.rotateMatrix = function(rotmat){
  this.uMVMatrix.rotateMatrix(rotmat);
  return this;
};

/**
  * [rotateQuaternion description]
  * @param  {Array} quaternion (returned from quaternion function)
  * @return {p5.Renderer3D}      [description]
*/
Renderer3D.rotateQuaternion = function(quat){
  this.uMVMatrix.rotateQuaternion(quat);
  return this;
 };

rotateMatrix = function() {
   //in webgl mode
  if(this._renderer.isP3D){
    this._renderer.rotateMatrix(arguments[0]);
  }
  else {
    throw 'not supported in p2d. Please use webgl mode';
  }
  return this;
};

rotateQuaternion = function() {
  //in webgl mode
  if(this._renderer.isP3D){
    this._renderer.rotateQuaternion(arguments[0]);
  }
  else {
    throw 'not supported in p2d. Please use webgl mode';
  }
  return this;
};

quaternionDot = function(w1,w2){
  return (w1[0]*w2[0] + w1[1]*w2[1] + w1[2]*w2[2] + w1[3]*w2[3]);
};

quaternionNorm = function(w){
  return this.quaternionDot(w,w);
};

quaternionDistance = function(w1,w2){
  var _quatDot = this.quaternionDot(w1,w2);
  return 1 - _quatDot*_quatDot;
};

quaternionAngleBetween = function(w1,w2){
  var _quatDot = this.quaternionDot(w1,w2);
  return Math.acos(2*_quatDot*_quatDot - 1);
};

quaternionConjugate = function(w){
  return [w[0],-w[1],-w[2],-w[3]];
};

quaternionMultiply = function(w1,w2){
  var _q0 = w1[0]*w2[0] - w1[1]*w2[1] - w1[2]*w2[2] - w1[3]*w2[3];
  var _q1 = w1[0]*w2[1] + w1[1]*w2[0] + w1[2]*w2[3] - w1[3]*w2[2];
  var _q2 = w1[0]*w2[2] - w1[1]*w2[3] + w1[2]*w2[0] + w1[3]*w2[1];
  var _q3 = w1[0]*w2[3] + w1[1]*w2[2] - w1[2]*w2[1] + w1[3]*w2[0];
  return [_q0,_q1,_q2,_q3];
};

_ondeviceorientation = function (e) {
  this._updatePRotations();
  this._setProperty('rotationX', e.beta ? e.beta : 0);
  this._setProperty('rotationY', e.gamma ? e.gamma : 0);
  this._setProperty('rotationZ', e.alpha ? e.alpha : 0);
  this._handleMotion();
};
