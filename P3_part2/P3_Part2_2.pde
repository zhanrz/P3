int numBoids = 50;
Vec2[] pos = new Vec2[numBoids];
Vec2[] vel = new Vec2[numBoids];
Vec2[] acc = new Vec2[numBoids];

float maxSpeed = 20;
float targetSpeed = 10;
float maxForce = 10;
float radius = 6;
float neighborDist = 50;
float desiredSeparation = 25; 

void setup() {
  size(900, 800);
  
  for (int i = 0; i < numBoids; i++) {
    pos[i] = new Vec2(200 + random(300), 200 + random(200));
    vel[i] = new Vec2(-1 + random(2), -1 + random(2));
    vel[i].normalize();
    vel[i].mul(maxSpeed);
  }
}

void draw() {
  background(255);
  stroke(0);
  
  float dt = 0.1;
  
  for (int i = 0; i < numBoids; i++) {
    acc[i] = new Vec2(0, 0);


    Vec2 sep = separate(i);   
    Vec2 ali = align(i);      
    Vec2 coh = cohesion(i);   

    sep.mul(1.5);  
    ali.mul(1.0);
    coh.mul(1.0);

    acc[i] = acc[i].plus(sep);
    acc[i] = acc[i].plus(ali);
    acc[i] = acc[i].plus(coh);

    pos[i] = pos[i].plus(vel[i].times(dt));
    vel[i] = vel[i].plus(acc[i].times(dt));

    if (vel[i].length() > maxSpeed) {
      vel[i] = vel[i].normalized().times(maxSpeed);
    }

    if (pos[i].x < 0) pos[i].x += width;
    if (pos[i].x > width) pos[i].x -= width;
    if (pos[i].y < 0) pos[i].y += height;
    if (pos[i].y > height) pos[i].y -= height;

    drawBoid(pos[i], vel[i]);
  }
}

Vec2 separate(int i) {
  Vec2 steer = new Vec2(0, 0);
  int count = 0;
  for (int j = 0; j < numBoids; j++) {
    float d = pos[i].distanceTo(pos[j]);
    if ((d > 0) && (d < desiredSeparation)) {
      Vec2 diff = pos[i].minus(pos[j]);
      diff.normalize();
      diff.div(d); 
      steer.add(diff);
      count++;
    }
  }
  if (count > 0) {
    steer.div((float)count);
  }

  if (steer.length() > 0) {
    steer.normalize();
    steer.mul(maxSpeed);
    steer.subtract(vel[i]);
    steer.clampToLength(maxForce);
  }
  return steer;
}

Vec2 align(int i) {
  Vec2 sum = new Vec2(0, 0);
  int count = 0;
  for (int j = 0; j < numBoids; j++) {
    float d = pos[i].distanceTo(pos[j]);
    if ((d > 0) && (d < neighborDist)) {
      sum.add(vel[j]);
      count++;
    }
  }
  if (count > 0) {
    sum.div((float)count);
    sum.normalize();
    sum.mul(maxSpeed);
    Vec2 steer = sum.minus(vel[i]);
    steer.clampToLength(maxForce);
    return steer;
  } else {
    return new Vec2(0, 0);
  }
}

Vec2 cohesion(int i) {
  Vec2 sum = new Vec2(0, 0);
  int count = 0;
  for (int j = 0; j < numBoids; j++) {
    float d = pos[i].distanceTo(pos[j]);
    if ((d > 0) && (d < neighborDist)) {
      sum.add(pos[j]);
      count++;
    }
  }
  if (count > 0) {
    sum.div((float)count);
    return seek(i, sum);
  } else {
    return new Vec2(0, 0);
  }
}

Vec2 seek(int i, Vec2 target) {
  Vec2 desired = target.minus(pos[i]);
  desired.normalize();
  desired.mul(maxSpeed);
  Vec2 steer = desired.minus(vel[i]);
  steer.clampToLength(maxForce);
  return steer;
}

void drawBoid(Vec2 position, Vec2 velocity) {
  pushMatrix();
  translate(position.x, position.y);
  rotate(velocity.heading() + PI / 2);
  fill(10, 120, 10);
  triangle(-radius, radius, radius, radius, 0, -radius * 2);
  popMatrix();
}


void mousePressed(){
  pos[0] = new Vec2(mouseX,mouseY);
  vel[0] = new Vec2(-1+random(2),-1+random(2));  
  vel[0].normalize();
  vel[0].mul(targetSpeed);
}
