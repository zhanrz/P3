

void setup(){
  size(640,480,P3D); 
}


PImage texture;
float cameraX = 320;
float cameraY = 240;
float cameraZ = 500;
float centerX = 320;
float centerY = 240;
float centerZ = 0;
float upX = 0;
float upY = 1;
float upZ = 0;


Vec2 root = new Vec2(0,0);


float l0 = 100; 
float a0 = 0.3; 


float l1 = 100;
float a1 = 0.3; 


float l2 = 100;
float a2 = 0.3; 

float l3 = 100; 
float a3 = 0.3; 

float l4 = 100; 
float a4 = 0.3; 


Vec2 start_l1,start_l2,start_l3, start_l4,endPoint;

float a0_min = -PI; float a0_max = PI;
float a1_min = -PI/2; float a1_max = PI/2;
float a2_min = -PI/2; float a2_max = PI/2;
float a3_min = -PI/2; float a3_max = PI/2;
float a4_min = -PI/2; float a4_max = PI/2;

//float a0_min = -Float.MAX_VALUE;
//float a0_max = Float.MAX_VALUE;
//float a1_min = -Float.MAX_VALUE;
//float a1_max = Float.MAX_VALUE;
//float a2_min = -Float.MAX_VALUE;
//float a2_max = Float.MAX_VALUE;
//float a3_min = -Float.MAX_VALUE;
//float a3_max = Float.MAX_VALUE;
//float a4_min = -Float.MAX_VALUE;
//float a4_max = Float.MAX_VALUE;


float maxAngleChange = 2; 

void solve() {
  Vec2 goal = new Vec2(mouseX, mouseY);
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  float old_a4 = a4, old_a3 = a3, old_a2 = a2, old_a1 = a1, old_a0 = a0;


  do {
    a4 = old_a4;
    startToGoal = goal.minus(start_l4);
    startToEndEffector = endPoint.minus(start_l4);
    dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
    dotProd = clamp(dotProd, -1, 1);
    angleDiff = acos(dotProd);
    if (cross(startToGoal, startToEndEffector) < 0)
      a4 += angleDiff;
    else
      a4 -= angleDiff;
    
  
    float angleChange = a4 - old_a4;
    if (Math.abs(angleChange) > maxAngleChange) {
      a4 = old_a4 + Math.signum(angleChange) * maxAngleChange;
    }
    fk();
    angleDiff *= .05;
  } while (armCircleCollision());

  do {
    a3 = old_a3;
    startToGoal = goal.minus(start_l3);
    startToEndEffector = endPoint.minus(start_l3);
    dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
    dotProd = clamp(dotProd, -1, 1);
    angleDiff = acos(dotProd);
    if (cross(startToGoal, startToEndEffector) < 0)
      a3 += angleDiff;
    else
      a3 -= angleDiff;

 
    float angleChange = a3 - old_a3;
    if (Math.abs(angleChange) > maxAngleChange) {
      a3 = old_a3 + Math.signum(angleChange) * maxAngleChange;
    }
    fk();
    angleDiff *= .05;
  } while (armCircleCollision());


  do {
    a2 = old_a2;
    startToGoal = goal.minus(start_l2);
    startToEndEffector = endPoint.minus(start_l2);
    dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
    dotProd = clamp(dotProd, -1, 1);
    angleDiff = acos(dotProd);
    if (cross(startToGoal, startToEndEffector) < 0)
      a2 += angleDiff;
    else
      a2 -= angleDiff;

 
    float angleChange = a2 - old_a2;
    if (Math.abs(angleChange) > maxAngleChange) {
      a2 = old_a2 + Math.signum(angleChange) * maxAngleChange;
    }
    fk();
    angleDiff *= .05;
  } while (armCircleCollision());

 
  do {
    a1 = old_a1;
    startToGoal = goal.minus(start_l1);
    startToEndEffector = endPoint.minus(start_l1);
    dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
    dotProd = clamp(dotProd, -1, 1);
    angleDiff = acos(dotProd);
    if (cross(startToGoal, startToEndEffector) < 0)
      a1 += angleDiff;
    else
      a1 -= angleDiff;

   
    float angleChange = a1 - old_a1;
    if (Math.abs(angleChange) > maxAngleChange) {
      a1 = old_a1 + Math.signum(angleChange) * maxAngleChange;
    }
    fk();
    angleDiff *= .05;
  } while (armCircleCollision());


  do {
    a0 = old_a0;
    startToGoal = goal.minus(root);
    if (startToGoal.length() < .0001) break;
    startToEndEffector = endPoint.minus(root);
    dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
    dotProd = clamp(dotProd, -1, 1);
    angleDiff = acos(dotProd);
    if (cross(startToGoal, startToEndEffector) < 0)
      a0 += angleDiff;
    else
      a0 -= angleDiff;


    float angleChange = a0 - old_a0;
    if (Math.abs(angleChange) > maxAngleChange) {
      a0 = old_a0 + Math.signum(angleChange) * maxAngleChange;
    }
    fk();
    angleDiff *= .05;
  } while (armCircleCollision());

  println("Angle 0:", a0, "Angle 1:", a1, "Angle 2:", a2, "Angle 3:", a3, "Angle 4:", a4);
}



float applyJointLimit(float angle, float min, float max) {
  return max(min(angle, max), min);
}

void fk(){
  start_l1 = new Vec2(cos(a0)*l0, sin(a0)*l0).plus(root);
  start_l2 = new Vec2(cos(a0+a1)*l1, sin(a0+a1)*l1).plus(start_l1);
  start_l3 = new Vec2(cos(a0+a1+a2)*l2, sin(a0+a1+a2)*l2).plus(start_l2);
  start_l4 = new Vec2(cos(a0+a1+a2+a3)*l3, sin(a0+a1+a2+a3)*l3).plus(start_l3);
  endPoint = new Vec2(cos(a0+a1+a2+a3+a4)*l4, sin(a0+a1+a2+a3+a4)*l4).plus(start_l4);
  
  a0 = applyJointLimit(a0, a0_min, a0_max);
  a1 = applyJointLimit(a1, a1_min, a1_max);
  a2 = applyJointLimit(a2, a2_min, a2_max);
  a3 = applyJointLimit(a3, a3_min, a3_max);
  a4 = applyJointLimit(a4, a4_min, a4_max);
}

float armW = 20;

void draw(){
  
  camera(cameraX, cameraY, cameraZ, centerX, centerY, centerZ, upX, upY, upZ);

  lights();

  fk();
  solve();
  
  background(250,250,250);
  
  fill(255, 0, 0); 
  ellipse(obstacleX, obstacleY, obstacleRadius * 2, obstacleRadius * 2);
  
  fill(0, 255, 0); 
  ellipse(obstacleX2, obstacleY2, obstacleRadius2 * 2, obstacleRadius2 * 2);
  

  fill(200,0,180);
  

  pushMatrix();
  translate(root.x,root.y);
  rotate(a0);
  rect(0, -armW/2, l0, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l1.x,start_l1.y);
  rotate(a0+a1);
  rect(0, -armW/2, l1, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l2.x,start_l2.y);
  rotate(a0+a1+a2);
  rect(0, -armW/2, l2, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l3.x, start_l3.y);
  rotate(a0+a1+a2+a3);
  rect(0, -armW/2, l3, armW);
  popMatrix();

  pushMatrix();
  translate(start_l4.x, start_l4.y);
  rotate(a0+a1+a2+a3+a4);
  rect(0, -armW/2, l4, armW);
  popMatrix();
  
  
}

void drawArmSegment(Vec2 start, float angle, float length, float width) {
  pushMatrix();
  translate(start.x, start.y);
  rotate(angle);

  int armSegments = 10;
  float segmentLength = length / armSegments;

  textureMode(NORMAL);
  noStroke();
  for (int j = 0; j < armSegments; j++) {
    float u1 = map(j, 0, armSegments, 0, 1);
    float u2 = map(j + 1, 0, armSegments, 0, 1);

    beginShape(TRIANGLE_STRIP);
    texture(texture);

    for (int i = 0; i <= 1; i++) {
      float v = map(i, 0, 1, 0, 1);
      float x = j * segmentLength - length / 2;
      float y = (i * 2 - 1) * width / 2;

      vertex(x, y, 0, u1, v);
      vertex(x + segmentLength, y, 0, u2, v);
    }

    endShape();
  }

  popMatrix();
}

void keyPressed() {

  if (key == 'w') cameraZ -= 10;
  if (key == 's') cameraZ += 10;
  if (key == 'a') cameraX -= 10;
  if (key == 'd') cameraX += 10;
  
}

float obstacleX = 200; 
float obstacleY = 300; 
float obstacleRadius = 35; 

float obstacleX2 = 400; 
float obstacleY2 = 100; 
float obstacleRadius2 = 45; 

boolean armCircleCollision() {
  if (checkCollisionWithObstacle(obstacleX, obstacleY, obstacleRadius)) return true;
  if (checkCollisionWithObstacle(obstacleX2, obstacleY2, obstacleRadius2)) return true;
  return false;
}

boolean checkCollisionWithObstacle(float obstacleX, float obstacleY, float obstacleRadius) {
  
  if (rectCircleCollision(endPoint.x, endPoint.y, l4, armW, a0 + a1 + a2 + a3 + a4, obstacleX, obstacleY, obstacleRadius)) return true;
    if (rectCircleCollision(start_l4.x, start_l4.y, l3, armW, a0 + a1 + a2 + a3, obstacleX, obstacleY, obstacleRadius)) return true;
    if (rectCircleCollision(start_l3.x, start_l3.y, l2, armW, a0 + a1 + a2, obstacleX, obstacleY, obstacleRadius)) return true;
      if (rectCircleCollision(start_l2.x, start_l2.y, l1, armW, a0 + a1, obstacleX, obstacleY, obstacleRadius)) return true;
        if (rectCircleCollision(start_l1.x, start_l1.y, l0, armW, a0, obstacleX, obstacleY, obstacleRadius)) return true;

  return false;
}


boolean rectCircleCollision(float rectX, float rectY, float rectWidth, float rectHeight, float rectAngle, float obstacleX, float obstacleY, float obstacleRadius)  {
  float cosA = cos(-rectAngle);
  float sinA = sin(-rectAngle);
  float localCircleX = cosA * (obstacleX - rectX) - sinA * (obstacleY - rectY);
  float localCircleY = sinA * (obstacleX - rectX) + cosA * (obstacleY - rectY);

  float closestX = clamp(localCircleX, -rectWidth / 2, rectWidth / 2);
  float closestY = clamp(localCircleY, -rectHeight / 2, rectHeight / 2);

  float distanceX = localCircleX - closestX;
  float distanceY = localCircleY - closestY;
  float distanceSquared = (distanceX * distanceX) + (distanceY * distanceY);

  return distanceSquared < (obstacleRadius * obstacleRadius);
}


boolean draggingObstacle1 = false;
boolean draggingObstacle2 = false;


void mousePressed() {
  if (dist(mouseX, mouseY, obstacleX, obstacleY) < obstacleRadius) {
    draggingObstacle1 = true;
  } else if (dist(mouseX, mouseY, obstacleX2, obstacleY2) < obstacleRadius2) {
    draggingObstacle2 = true;
  }
}

void mouseReleased() {
  draggingObstacle1 = false;
  draggingObstacle2 = false;
}

void mouseDragged() {
  if (draggingObstacle1) {
    obstacleX = mouseX;
    obstacleY = mouseY;
  } else if (draggingObstacle2) {
    obstacleX2 = mouseX;
    obstacleY2 = mouseY;
  }
}



////-----------------
//// Vector Library
////-----------------


public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}


Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max){
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
