static int maxNumAgents = 3;
int numAgents = 3;

float k_goal = 5;  
float k_avoid = 10; 
float agentRad = 40;
float goalSpeed = 100;
float obstacleRad = 50;  

Vec2[] agentPos = new Vec2[maxNumAgents];
Vec2[] agentVel = new Vec2[maxNumAgents];
Vec2[] agentAcc = new Vec2[maxNumAgents];

Vec2[] goalPos = new Vec2[maxNumAgents];

Vec2 obstaclePos = new Vec2(350, 325); 

void setup(){
  size(850,650);

  agentPos[0] = new Vec2(220,610);
  agentPos[1] = new Vec2(320,650);
  agentPos[2] = new Vec2(320,420);
  goalPos[0] = new Vec2(200,420);
  goalPos[1] = new Vec2(120,120);
  goalPos[2] = new Vec2(220,220);

  for (int i = 0; i < numAgents; i++){
    agentVel[i] = goalPos[i].minus(agentPos[i]);
    if (agentVel[i].length() > 0)
      agentVel[i].setToLength(goalSpeed);
  }
}

float computeTTC(Vec2 pos1, Vec2 vel1, float radius1, Vec2 pos2, Vec2 vel2, float radius2){

  Vec2 relativePos = pos2.minus(pos1);
  Vec2 relativeVel = vel2.minus(vel1);
  return rayCircleIntersectTime(relativePos, radius1 + radius2, new Vec2(0,0), relativeVel);
}

Vec2 computeAgentForces(int id){
  Vec2 acc = new Vec2(0,0);


  Vec2 goalVel = goalPos[id].minus(agentPos[id]).normalized().times(goalSpeed);
  Vec2 goalForce = goalVel.minus(agentVel[id]).times(k_goal);
  acc.add(goalForce);


  for (int j = 0; j < numAgents; j++){
    if (j != id) {
      float distanceToAgent = agentPos[id].distanceTo(agentPos[j]);
      if (distanceToAgent < agentRad * 2) { 
        Vec2 escapeDirection = agentPos[id].minus(agentPos[j]).normalized();
        Vec2 newGoalVel = escapeDirection.times(goalSpeed);
        Vec2 newGoalForce = newGoalVel.minus(agentVel[id]).times(k_goal);
        acc.add(newGoalForce); 
      }
    }
  }


  float distanceToObstacle = agentPos[id].distanceTo(obstaclePos);
  if (distanceToObstacle < agentRad + obstacleRad) { 
    Vec2 escapeDirection = agentPos[id].minus(obstaclePos).normalized();
    Vec2 newGoalVel = escapeDirection.times(goalSpeed);
    Vec2 newGoalForce = newGoalVel.minus(agentVel[id]).times(k_goal);
    acc = newGoalForce; 
  }

  return acc;
}

void moveAgent(float dt){
  for (int i = 0; i < numAgents; i++){
    agentAcc[i] = computeAgentForces(i);
    agentVel[i].add(agentAcc[i].times(dt));
    agentPos[i].add(agentVel[i].times(dt));
  }
}

boolean paused = true;
void draw(){
  background(255,255,255);
  if (!paused){
    moveAgent(1.0/frameRate);
  }


  fill(255,150,50);
  for (int i = 0; i < numAgents; i++){
    rect(goalPos[i].x-10, goalPos[i].y-10, 20, 20);
  }


  fill(20,200,150);
  for (int i = 0; i < numAgents; i++){
    circle(agentPos[i].x, agentPos[i].y, agentRad*2);
  }


  fill(100, 100, 100); 
  circle(obstaclePos.x, obstaclePos.y, obstacleRad * 2);
}

void keyPressed(){
  if (key == ' ') paused = !paused;
}


float rayCircleIntersectTime(Vec2 center, float r, Vec2 l_start, Vec2 l_dir){
 

  Vec2 toCircle = center.minus(l_start);
 

  float a = l_dir.length()*l_dir.length();
  float b = -2*dot(l_dir,toCircle); 
  float c = toCircle.lengthSqr() - (r*r); 
 
  float d = b*b - 4*a*c; 
 
  if (d >=0 ){
 
    float t = (-b - sqrt(d))/(2*a); 
    if (t >= 0) return t;
    return -1;
  }
 
  return -1; 
}
