
var   b2Vec2 = Box2D.Common.Math.b2Vec2
     , b2BodyDef = Box2D.Dynamics.b2BodyDef
     , b2Body = Box2D.Dynamics.b2Body
     , b2FixtureDef = Box2D.Dynamics.b2FixtureDef
     , b2Fixture = Box2D.Dynamics.b2Fixture
     , b2World = Box2D.Dynamics.b2World
     , b2MassData = Box2D.Collision.Shapes.b2MassData
     , b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
     , b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
     , b2Shape = Box2D.Collision.Shapes.b2Shape
     , b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef
     , b2WeldJointDef = Box2D.Dynamics.Joints.b2WeldJointDef
     , b2FilterData = Box2D.Dynamics.b2FilterData
     , b2ContactListener = Box2D.Dynamics.b2ContactListener;

var canv = document.getElementById('canvas');
var ctx = canv.getContext('2d');
var cWidth = canv.width = 1054;
var cHeight = canv.height = 632;
var mainLoopPaused = false;

min = Math.min;
max = Math.max;
round = Math.round;
floor = Math.floor;
random = Math.random;
cos = Math.cos;
sin = Math.sin;
abs = Math.abs;
pow = Math.pow;
sqrt = Math.sqrt;
PI = Math.PI;

function createBall(world, x, y, radius, fixed, density) {

    radius = 20;

    var bodyDef = new b2BodyDef;
    var fixDef = new b2FixtureDef;

    fixDef.density = density==undefined ? 1 : density;
    fixDef.friction = 5;
    fixDef.restitution = 0.5;

    bodyDef.type = fixed ? b2Body.b2_staticBody : b2Body.b2_dynamicBody;

    fixDef.shape = new b2CircleShape(radius);

    bodyDef.position.x = x;
    bodyDef.position.y = y;

    world.CreateBody(bodyDef).CreateFixture(fixDef);
    return world.GetBodyList();
}

function createPolygon(world, x, y, points, fixed, density) {

    var bodyDef = new b2BodyDef;
    var fixDef = new b2FixtureDef;

    fixDef.density = density==undefined ? 1 : density;
    fixDef.friction = 15;
    fixDef.restitution = 1;

    bodyDef.type = fixed ? b2Body.b2_staticBody : b2Body.b2_dynamicBody;

    fixDef.shape = new b2PolygonShape;
    fixDef.shape.SetAsArray(
        points.map( function (point) {
            return new b2Vec2(point.x, point.y);
        })
    );

    bodyDef.position.x = x;
    bodyDef.position.y = y;

    world.CreateBody(bodyDef).CreateFixture(fixDef);
    return world.GetBodyList();
}

function createBox(world, x, y, width, height, r, fixed, density) {
    if (r == 0 || r == undefined) {
        vtx = [ {'x':-width/2, 'y':-height/2},
            {'x':width/2, 'y':-height/2},
            {'x':width/2, 'y':height/2},
            {'x':-width/2, 'y':height/2}];
        return createPolygon(world, x+(width/2),y+(height/2), vtx, fixed,density);
    } else {
        var cosr = cos(r), sinr = sin(r);
        var dx = width/2, dy = height/2;
        vtx = [ {'x':-dx*cosr+dy*sinr, 'y':-dx*sinr-dy*cosr},
            {'x':dx*cosr+dy*sinr, 'y':dx*sinr-dy*cosr},
            {'x':dx*cosr-dy*sinr, 'y':dx*sinr+dy*cosr},
            {'x':-dx*cosr-dy*sinr, 'y':-dx*sinr+dy*cosr}];
        return createPolygon(world, x+(width/2),y+(height/2), vtx, fixed,density);
    }
}

function initWalls(world, w,h,t) {
    // Create the floor
    var floor = createBox(world, 0,-t/2,w,t,0,true);
    floor.SetUserData('floor');
    // Create the left wall
    var l_wall = createBox(world, -t/2,0,t,h,0,true);
    l_wall.SetUserData('l_wall');
    // Create the right wall
    var r_wall = createBox(world, w-t/2,0,t,h,0,true);
    r_wall.SetUserData('r_wall');

    return {'floor':floor, 'l_wall':l_wall, 'r_wall':r_wall};
}

function translateBody(dx, dy) {
    (Object.keys(body)).forEach(function(elem) {
        var pos = body[elem].GetPosition();
        body[elem].SetType(b2Body.b2_staticBody);
        body[elem].SynchronizeTransform(new b2Vec2(pos.x+dx,pos.y+dy),0);
        body[elem].SetType(b2Body.b2_dynamicBody);
    });
}

var timestep = 60;
var freq = 1/timestep;
var gravity = new b2Vec2(0, -75);
var world = new b2World(gravity, true);
var worldWidth = 500;
var worldHeight = 300;
var environment = initWalls(world,worldWidth,worldHeight,24);
var fakeWorld = undefined;

var distData = [];
var walkData = [];
var stepData = [];
var recordLoopId = undefined;

function xToWorld(x) {
    return worldWidth*x/cWidth;
}

function yToWorld(y) {
    return worldHeight*(cHeight-y)/cHeight;
}

function xToCanvas(x) {
    // The x coordinate starts at 0 and ends at 500
    return cWidth*x/worldWidth;
}

function yToCanvas(y) {
    // The y coordinate starts at 500 and ends at 0
    return cHeight*(worldHeight-y)/worldHeight;
}

function toCanvas(x,y) {
    return [xToCanvas(x),yToCanvas(y)];
}

function hasContact(body1, body2) {
    cList = body1.GetContactList();
    while(cList != null) {
        if (cList.other == body2) {
            return true;
        }
        cList = cList.next;
    }
    return false;
}

/**
  Return 1 if near upper limit, -1 if near lower limit,
  and 0 if not near a limit.
*/
function kneeAtLimit(kneeJoint) {
    if (kneeJoint.GetJointAngle() < kneeLimits[0]+0.2) {
        return -1;
    } else if (kneeJoint.GetJointAngle() > kneeLimits[1]-0.2) {
        return 1;
    } else {
       return 0;
    }
}

/**
  Return 1 if near upper limit, -1 if near lower limit,
  and 0 if not near a limit.
*/
function hipAtLimit(hipJoint) {
    if (hipJoint.GetJointAngle() < hipLimits[0]+0.3) {
        return -1;
    } else if (hipJoint.GetJointAngle() > hipLimits[1]-0.3) {
        return 1;
    } else {
       return 0;
    }
}

var aimode = false;
var showAIDetails = false;
var drawWorld = true;
var recordingSteps = false;

var init = false;
var keyMasks = {q:1,w:2,o:4,p:8};
var keyState = 0;
var keyEventCodes = {80:'p',79:'o',87:'w',81:'q',32:' '};
var action_strings = {0:' ',1:'Q',2:'W',4:'O',6:'WO',8:'P',9:'QP'};
var maintainLeftHipStability = true;
var maintainLeftKneeStability = true;
var maintainRightHipStability = true;
var maintainRightKneeStability = true;

var joint = {};
var body = {};

var l_kneeAngle = 0.175, r_kneeAngle = 0.175;
var l_hipAngle = -0.25, r_hipAngle = 0.5;
var l_hip_rotate_speed = 3;
var r_hip_rotate_speed = 3;
var l_knee_rotate_speed = 3;
var r_knee_rotate_speed = 3;
var hipLimits = [-1,1];
var kneeLimits = [-0.25,1];

var elapsedTime = 0.0;
var totalDistTraveled = 0.0;
var farthestDistTraveled = 0.0;
var curX = 0.0
var requestTeleport = false;
var curVelX = 0.0;
var prevVelX = 0.0;

var autoReset = true;
var requestReset = false;
var respawning = false;
var walkDelay = false;

var legalKeyStates = [true,true,true,false,true,false,true,false,true,
                    true,false,false,false,false,false,false];

var hipJointAngle = 0.0;
var prevHipJointAngle = 0.0;
var step_phase = [false,false];
var stepBeginAngle = NaN;
var stepBackLeg = undefined;
var stepForwardLeg = undefined;
var stepBackJoint = undefined;
var totalStepsTraveled = 0;
var lastStepX = 0;
var stepDistances = [];

var deathCount = 0;
var scoreCheckpointDist = 2;
var scorePenaltyDist = 2;

document.onkeydown = function(event) {
    var c = keyEventCodes[event.keyCode];
    if (c == ' ') {
        if (aimode) {
            advance_step();
        } else if (elapsedTime > 1.5) {
            resetRunner();
        }
    }
    var m = keyMasks[c];
    if (m !== undefined && legalKeyStates[keyState | m]) {
        keyState = keyState | m;
    }
}

function handleKeyPressed(keymask) {
    switch(keymask) {
        case 1: // Q
            handleQPressed();
        break;

        case 2: // W
            handleWPressed();
        break;

        case 4: // O
            handleOPressed();
        break;

        case 8: // P
            handlePPressed();
        break;
    }
}

function handleQPressed() {
    if (maintainLeftHipStability) {
        maintainLeftHipStability = false;
    }
    joint.l_hip.SetMotorSpeed(l_hip_rotate_speed);
    joint.r_hip.SetMotorSpeed(-r_hip_rotate_speed);
}

function handleWPressed() {
    if (maintainRightHipStability) {
        maintainRightHipStability = false;
    }
    joint.l_hip.SetMotorSpeed(-l_hip_rotate_speed);
    joint.r_hip.SetMotorSpeed(r_hip_rotate_speed);
}

function handleOPressed() {
    if (maintainLeftKneeStability || maintainRightKneeStability) {
        maintainLeftKneeStability = false;
        maintainRightKneeStability = false;
    }
    joint.l_knee.SetMotorSpeed(l_knee_rotate_speed);
    joint.r_knee.SetMotorSpeed(-r_knee_rotate_speed);
}

function handlePPressed() {
    if (maintainRightKneeStability || maintainLeftKneeStability) {
        maintainLeftKneeStability = false;
        maintainRightKneeStability = false;
    }
    joint.r_knee.SetMotorSpeed(r_knee_rotate_speed);
    joint.l_knee.SetMotorSpeed(-l_knee_rotate_speed);
}

function handleKeyReleased(keymask) {
    switch(keymask) {
        case 1: // Q
            handleQReleased();
        break;

        case 2: // W
            handleWReleased();
        break;

        case 4: // O
            handleOReleased();
        break;

        case 8: // P
            handlePReleased();
        break;
    }
}

function handleQReleased() {
    maintainLeftHipStability = true;
    joint.l_hip.SetMotorSpeed(0);
    joint.r_hip.SetMotorSpeed(0);
    l_hipAngle = joint.l_hip.GetJointAngle();
    r_hipAngle = joint.r_hip.GetJointAngle();
}

function handleWReleased() {
    maintainRightHipStability = true;
    joint.r_hip.SetMotorSpeed(0);
    joint.l_hip.SetMotorSpeed(0);
    r_hipAngle = joint.r_hip.GetJointAngle();
    l_hipAngle = joint.l_hip.GetJointAngle();
}

function handleOReleased() {
    maintainLeftKneeStability = true;
    maintainRightKneeStability = true;
    joint.l_knee.SetMotorSpeed(0);
    joint.r_knee.SetMotorSpeed(0);
    l_kneeAngle = joint.l_knee.GetJointAngle();
    r_kneeAngle = joint.r_knee.GetJointAngle();
}

function handlePReleased() {
    maintainLeftKneeStability = true;
    maintainRightKneeStability = true;
    joint.r_knee.SetMotorSpeed(0);
    joint.l_knee.SetMotorSpeed(0);
    l_kneeAngle = joint.l_knee.GetJointAngle();
    r_kneeAngle = joint.r_knee.GetJointAngle();
}

document.onkeyup = function(event) {
    var c = keyEventCodes[event.keyCode];
    var m = keyMasks[c];
    if (m !== undefined && legalKeyStates[keyState & ~m]) {
        keyState = keyState & ~m;
        handleKeyReleased(m);
    }
}

function updateKeyState(nextState) {
    var m = 1;
    for (var i = 0; i < 4; i++) {
        var delta = (nextState&m)-(keyState&m);
        if (delta == -m) {
            handleKeyReleased(m)
        }
        m *= 2;
    }
    keyState = nextState;
}

var listener = new b2ContactListener;
listener.BeginContact = function(contact) {
    /*
    body_A = contact.GetFixtureA().GetBody();
    body_B = contact.GetFixtureB().GetBody();
    if (body_A == environment.floor) {
        // console.log(body_B.GetUserData() + ' is touching the floor!');
        // body_B.ApplyImpulse(new b2Vec2(-300000,0),body_B.GetWorldCenter());
    } else if (body_B == environment.floor) {
        // console.log(body_A.GetUserData() + ' is touching the floor!');
        // body_A.ApplyImpulse(new b2Vec2(-300000,0),body_A.GetWorldCenter());
    }
    */
}
listener.PreSolve = function(contact, oldManifold) {
    body_A = contact.GetFixtureA().GetBody();
    body_B = contact.GetFixtureB().GetBody();
    if (body_A == environment.r_wall || body_B == environment.r_wall) {
        requestTeleport = true;
    } else if (autoReset) {
        if (body_A == environment.floor) {
            if (body_B !== body.ll_leg && body_B !== body.ul_leg &&
             body_B !== body.lr_leg && body_B !== body.ur_leg) {
                requestReset = true;
            }
        } else if (body_B == environment.floor) {
            if (body_A !== body.ll_leg && body_A !== body.ul_leg &&
             body_A !== body.lr_leg && body_A !== body.ur_leg) {
                requestReset = true;
            }
        }
    }
}
world.SetContactListener(listener);

function shiftBodyX(dx,dy) {
    Object.keys(body).forEach(function(part) {
        var b = body[part];
        var newPos = new b2Vec2(b.GetWorldCenter().x+dx,b.GetWorldCenter().y+dy);
        b.SetPosition(newPos);
    });
}

function lockRevoluteJoint(joint,torque) {
    torque = torque==undefined ? Infinity : torque;
    joint.SetMaxMotorTorque(torque);
    joint.SetMotorSpeed(0);
    joint.EnableMotor(true);
}

function resetRunner() {

    if (init) {
        // Has already been initialized at least once
        Object.keys(body).forEach( function(part) {
            world.DestroyBody(body[part]);
        });
        Object.keys(joint).forEach( function(part) {
            world.DestroyJoint(joint[part]);
        });
        recordedMoves = [{'key':'none', 'type':'initialization', 'time':0}];
        elapsedTime = 0.0;
        totalDistTraveled = 0.0;
        totalStepsTraveled = 0;
    }

    maintainLeftHipStability = true;
    maintainLeftKneeStability = true;
    maintainRightHipStability = true;
    maintainRightKneeStability = true;

    joint = {};
    body = {};

    l_kneeAngle = 0.175, r_kneeAngle = 0.175;
    l_hipAngle = -0.25, r_hipAngle = 0.5;

    init = true;

    // Create all body parts
    var head = createBall(world,250,160,25,false,0.1);
    var l_arm = createBox(world,202,126,40,8,0,false,0.1);
    var ul_leg = createBox(world,248,44,10,40,PI/6,false,10);
    var ll_leg = createBox(world,248,16,10,40,-PI/6,false,10);
    var torso = createBox(world,234,68,32,70,0,false,5);
    var r_arm = createBox(world,258,126,40,8,0,false,0.1);
    var ur_leg = createBox(world,248,44,10,40,PI/6,false,10);
    var lr_leg = createBox(world,248,16,10,40,-PI/6,false,10);

    curVelX = 0.0;
    // var prevVelX = 0.0;

    // Track joints and body components
    body.head = head;
    body.torso = torso;
    body.l_arm = l_arm;
    body.r_arm = r_arm;
    body.ll_leg = ll_leg;
    body.ul_leg = ul_leg;
    body.lr_leg = lr_leg;
    body.ur_leg = ur_leg;
    head.SetUserData('head');
    torso.SetUserData('torso');
    l_arm.SetUserData('l_arm');
    r_arm.SetUserData('r_arm');
    ll_leg.SetUserData('ll_leg');
    ul_leg.SetUserData('ul_leg');
    lr_leg.SetUserData('lr_leg');
    ur_leg.SetUserData('ur_leg');

    // Connect head and torso
    var neck_jointDef = new b2WeldJointDef();
    var neck_anchor = head.GetWorldCenter();
    neck_anchor.y = neck_anchor.y - 20;
    neck_jointDef.Initialize(head, torso, neck_anchor);
    var neck_joint = world.CreateJoint(neck_jointDef);

    // Connect left arm to torso
    var l_arm_jointDef = new b2RevoluteJointDef();
    var l_arm_anchor = l_arm.GetWorldCenter();
    l_arm_anchor.x = l_arm_anchor.x + 15;
    l_arm_jointDef.Initialize(l_arm, torso, l_arm_anchor);
    var l_arm_joint = world.CreateJoint(l_arm_jointDef);

    // Connect right arm to torso
    var r_arm_jointDef = new b2RevoluteJointDef();
    var r_arm_anchor = r_arm.GetWorldCenter();
    r_arm_anchor.x = r_arm_anchor.x - 15;
    r_arm_jointDef.Initialize(r_arm, torso, r_arm_anchor);
    var r_arm_joint = world.CreateJoint(r_arm_jointDef);

    // Connect upper, lower left leg
    var l_knee_jointDef = new b2RevoluteJointDef();
    var l_knee_anchor = ul_leg.GetWorldCenter();
    l_knee_anchor.x = l_knee_anchor.x + 8.25;
    l_knee_anchor.y = l_knee_anchor.y - 14.3;
    l_knee_jointDef.Initialize(ul_leg, ll_leg, l_knee_anchor);
    var l_knee_joint = world.CreateJoint(l_knee_jointDef);

    // Connect upper, lower right leg
    var r_knee_jointDef = new b2RevoluteJointDef();
    var r_knee_anchor = ur_leg.GetWorldCenter();
    r_knee_anchor.x = r_knee_anchor.x + 8.25;
    r_knee_anchor.y = r_knee_anchor.y - 14.3;
    r_knee_jointDef.Initialize(ur_leg, lr_leg, r_knee_anchor);
    var r_knee_joint = world.CreateJoint(r_knee_jointDef);

    // Attach left, right legs to torso
    var l_hip_jointDef = new b2RevoluteJointDef();
    var l_hip_anchor = ul_leg.GetWorldCenter();
    l_hip_anchor.x = l_hip_anchor.x - 12;
    l_hip_anchor.y = l_hip_anchor.y + 26;
    l_hip_jointDef.Initialize(torso, ul_leg, l_hip_anchor);
    var l_hip_joint = world.CreateJoint(l_hip_jointDef);

    var r_hip_jointDef = new b2RevoluteJointDef();
    var r_hip_anchor = ur_leg.GetWorldCenter();
    r_hip_anchor.x = r_hip_anchor.x - 12;
    r_hip_anchor.y = r_hip_anchor.y + 26;
    r_hip_jointDef.Initialize(torso, ur_leg, r_hip_anchor);
    var r_hip_joint = world.CreateJoint(r_hip_jointDef);

    // body.l_foot = l_foot;
    // body.r_foot = r_foot;
    joint.neck = neck_joint;
    joint.l_arm = l_arm_joint;
    joint.r_arm = r_arm_joint;
    joint.l_hip = l_hip_joint;
    joint.r_hip = r_hip_joint;
    joint.l_knee = l_knee_joint;
    joint.r_knee = r_knee_joint;

    // Prevent arms, legs from colliding with each other
    setFilterGroup([l_arm,r_arm,environment.floor],-1);
    setFilterGroup([torso,ul_leg,ur_leg,ll_leg,lr_leg],-2);

    // Stiffen hip, arm and knee joints
    lockRevoluteJoint(l_knee_joint);
    lockRevoluteJoint(r_knee_joint);
    lockRevoluteJoint(l_hip_joint);
    lockRevoluteJoint(r_hip_joint);
    lockRevoluteJoint(l_arm_joint,8000);
    lockRevoluteJoint(r_arm_joint,8000);

    setInterval(function() {
        if (!mainLoopPaused) {
            if (maintainLeftHipStability) {
                var r = l_hip_joint.GetJointAngle();
                l_hip_joint.SetMotorSpeed(2*(l_hipAngle - r));
            }
            if (maintainRightHipStability) {
                r = r_hip_joint.GetJointAngle();
                r_hip_joint.SetMotorSpeed(2*(r_hipAngle - r));
            }
            if (maintainLeftKneeStability) {
                var r = l_knee_joint.GetJointAngle();
                l_knee_joint.SetMotorSpeed(2*(l_kneeAngle - r));
            }
            if (maintainRightKneeStability) {
                r = r_knee_joint.GetJointAngle();
                r_knee_joint.SetMotorSpeed(2*(r_kneeAngle - r));
            }
        }
    },100);

    l_hip_joint.EnableLimit(true);
    l_hip_joint.SetLimits(hipLimits[0],hipLimits[1]);   // -1 and 1
    r_hip_joint.EnableLimit(true);
    r_hip_joint.SetLimits(hipLimits[0],hipLimits[1]);
    l_knee_joint.EnableLimit(true);
    l_knee_joint.SetLimits(kneeLimits[0],kneeLimits[1]);    // -0.25 and 1
    r_knee_joint.EnableLimit(true);
    r_knee_joint.SetLimits(kneeLimits[0],kneeLimits[1]);

    curX = getHipBaseX();
}

function setFilterGroup(elems, fIndex) {
    elems.forEach(function(elem) {
        var fList = elem.GetFixtureList();
        var filterData = fList.GetFilterData();
        filterData.groupIndex = fIndex;
        fList.SetFilterData(filterData);
    });
}

/**
  Gets the position on the x axis of the ragdoll's hips. We use
  this as the true forward velocity of the ragdoll instead of
  the center of the torso, since we don't want to reward falling
  forward quickly (at least, not as much as before).
*/
function getHipBaseX() {
    var theta = body.torso.GetAngle();
    var x = body.torso.GetPosition().x;
    return x - 35*sin(theta);
}

function handleInput() {
    switch (keyState) {

        case 1:
            handleQPressed();
        break;

        case 2:
            handleWPressed();
        break;

        case 3:
            handleQPressed();
            handleWPressed();
        break;

        case 4:
            handleOPressed();
        break;

        case 5:
            handleQPressed();
            handleOPressed();
        break;

        case 6:
            handleWPressed();
            handleOPressed();
        break;

        case 7:
            handleQPressed();
            handleWPressed();
            handleOPressed();
        break;

        case 8:
            handlePPressed();
        break;

        case 9:
            handleQPressed();
            handlePPressed();
        break;

        case 10:
            handleWPressed();
            handlePPressed();
        break;

        case 11:
            handleQPressed();
            handleWPressed();
            handlePPressed();
        break;

        case 12:
            handleOPressed();
            handlePPressed();
        break;

        case 13:
            handleQPressed();
            handleOPressed();
            handlePPressed();
        break;

        case 14:
            handleWPressed();
            handleOPressed();
            handlePPressed();
        break;

        case 15:
            handleQPressed();
            handleWPressed();
            handleOPressed();
            handlePPressed();
        break;
    }
}

function getFootY(foot) {
    return (foot.GetUserData()=='ll_leg' ? getLeftFootY() : getRightFootY());
}

function getLeftFootY(bd) {
    bd = (bd==undefined) ? body : bd;
    return bd.ll_leg.GetPosition().y - (20 * cos(bd.ll_leg.GetAngle()));
}

function getRightFootY(bd) {
    bd = (bd==undefined) ? body : bd;
    return bd.lr_leg.GetPosition().y - (20 * cos(bd.lr_leg.GetAngle()));
}

// function getFootX(foot) {
//     return (foot.GetUserData()=='ll_leg' ? getLeftFootX() : getRightFootX());
// }

// function getLeftFootX(bd) {
//     bd = (bd==undefined) ? body : bd;
//     return bd.ll_leg.GetPosition().x + (20 * sin(bd.ll_leg.GetAngle()));
// }

// function getRightFootX(bd) {
//     bd = (bd==undefined) ? body : bd;
//     return bd.lr_leg.GetPosition().x + (20 * sin(bd.lr_leg.GetAngle()));
// }

function draw(node) {

    var pos = node.GetPosition();
    var fList = node.GetFixtureList();

    if (fList !== null) {

        var shape = fList.GetShape();
        var shapeType = shape.GetType();

        if (shapeType == b2Shape.e_circleShape) {
            ctx.beginPath();
            ctx.arc(xToCanvas(pos.x), yToCanvas(pos.y), 40, 0, 2 * PI, false);
            ctx.fillStyle = '#FFF3C3';
            ctx.fill();
            ctx.lineWidth = 5;
            ctx.strokeStyle = '#003300';
            ctx.stroke();
        } else {
            ctx.beginPath();

            var vtx = shape.m_vertices;
            var r = node.GetAngle();
            var sinr = sin(r), cosr = cos(r);
            var x0 = (vtx[0].x*cosr-vtx[0].y*sinr), y0 = (vtx[0].x*sinr+vtx[0].y*cosr);

            ctx.moveTo(xToCanvas(pos.x + x0), yToCanvas(pos.y+ y0));

            for (var i = 1; i < vtx.length; i++) {
                ctx.lineTo(xToCanvas(pos.x+(vtx[i].x*cosr-vtx[i].y*sinr)),
                    yToCanvas(pos.y+(vtx[i].x*sinr+vtx[i].y*cosr)));
            }
            ctx.lineTo(xToCanvas(pos.x + x0), yToCanvas(pos.y + y0));

            ctx.fillStyle = '#FFF3C3';
            ctx.fill();
            ctx.lineWidth = 5;
            ctx.strokeStyle = '#003300';
            ctx.stroke();
        }
    }
}

// function startRecording() {
//     recordLoopId = setInterval(record,50);
// }

// function endRecording(save,dist) {

//     clearInterval(recordLoopId);
//     if (save) {
//         data = []
//         stepData.slice(0,stepData.length-3).forEach(function(d) {
//             walkData.push(d);
//         })
//         console.log('SAVED WALK DATA.')
//     }
//     stepData = [];
// }

// function record() {
//     s = extractState(body,joint,environment,null,0)
//     stepData.push([stateVector(s),keyState])
// }

function mainLoop() {

    if (!mainLoopPaused) {
        ctx.clearRect(0,0,cWidth,cHeight);
        var node = world.GetBodyList();
        if (drawWorld) {
            while (node.GetNext() !== null) {
                draw(node);
                node = node.GetNext();
            }
            // if (fakeWorld !== undefined && fakeWorld != world) {
            //     var _node = fakeWorld.GetBodyList();
            //     while(_node.GetNext() !== null) {
            //         draw2(_node);
            //         _node = _node.GetNext();
            //     }
            // }
        }
        world.Step(freq,2,2);
        handleInput();
        elapsedTime += freq;
        if (requestTeleport) {
            requestTeleport = false;
            lastStepX = 0
            shiftBodyX(-300,0);
            curX = getHipBaseX();
        }
        var newX = getHipBaseX();
        var dx = (newX - curX)/25
        totalDistTraveled += dx;
        farthestDistTraveled = max(farthestDistTraveled,totalDistTraveled);
        curX = newX;
        prevVelX = curVelX;
        curVelX = dx/freq;

        if (dx > 0) {
            scoreCheckpointDist -= dx;
            if (scoreCheckpointDist <= 0) {
                if (aimode) {
                    notifyDistTraveled();
                }
                scoreCheckpointDist = 2;
                scorePenaltyDist = 2;
            }
        } else {
            scorePenaltyDist += dx;
            if (scorePenaltyDist <= 0) {
                if (aimode) {
                    notifyBackwardsTraveled();
                }
                scoreCheckpointDist = 2;
                scorePenaltyDist = 2;
            }
        }

        prevHipJointAngle = hipJointAngle;
        hipJointAngle = joint.r_hip.GetJointAngle() - joint.l_hip.GetJointAngle();

        if (!step_phase[0]) {
            if (hipJointAngle < -1.5 && hasContact(body.lr_leg,environment.floor)) {
                // right foot backwards
                lastStepX = getHipBaseX();
                stepBeginAngle = hipJointAngle;
                stepBackLeg = body.lr_leg;
                stepBackJoint = joint.r_knee;
                stepForwardLeg = body.ll_leg;
                step_phase[0] = true;
                if (aimode) {
                    notifyStepBegun();
                }
                if (recordingSteps) {
                    startRecording();
                    console.log('(+) WALK INITIATED');
                }
            } else if (hipJointAngle > 1.5 && hasContact(body.ll_leg,environment.floor)) {
                // left foot backwards
                lastStepX = getHipBaseX();
                stepBeginAngle = hipJointAngle;
                stepBackLeg = body.ll_leg;
                stepBackJoint = joint.l_knee;
                stepForwardLeg = body.lr_leg;
                step_phase[0] = true;
                if (aimode) {
                    notifyStepBegun();
                }
                if (recordingSteps) {
                    startRecording();
                    console.log('(+) WALK INITIATED');
                }
            }
        }

        if (step_phase[0] && !step_phase[1] && hipJointAngle*prevHipJointAngle < 0) {
            step_phase[1] = true;
            if (aimode) {
                notifyHalfStep();
            }
            if (recordingSteps) {
                console.log('(+) HALF STEP DETECTED');
            }
        }

        if (step_phase[0] && step_phase[1]) {
            if (hipJointAngle*stepBeginAngle < -2.5 &&
            getFootY(stepBackLeg) < 20 &&
            abs(body.torso.GetAngle()+0.25) < 0.4) {

                var cBaseHipX = getHipBaseX();
                var dist = cBaseHipX - lastStepX;

                if (recordingSteps) {
                    endRecording((dist>0),dist);
                }

                // back foot is now forwards
                totalStepsTraveled++;
                stepBeginAngle = hipJointAngle;
                stepBackLeg = (stepBackLeg == body.ll_leg ? body.lr_leg : body.ll_leg);
                stepBackJoint = (stepBackLeg == joint.l_knee ? joint.r_knee : joint.l_knee);
                step_phase[0] = true;
                step_phase[1] = false;
                if (aimode) {
                    notifyStepComplete(dist);
                }

                if (recordingSteps) {
                    console.log('(+) STEP DETECTED: DISTANCE OF ' + dist);
                    startRecording();
                }

                stepDistances.push(dist);
                lastStepX = cBaseHipX % 500;
            }
        }

        ctx.font = '20pt Calibri';
        ctx.fillStyle = 'black';
        
        ctx.fillText('Best Distance: ' + round(farthestDistTraveled) + ' m',100,50);
        ctx.fillText('Total Distance: ' + round(totalDistTraveled) + ' m',100,100);
        ctx.fillText('Keystate: ' + action_strings[keyState],100,150);

        if (aimode && showAIDetails) {
            ctx.fillText('Iterations: ' + iterations,100,200);
        }

        if (body.head.GetPosition().y < 75 && aimode) {
            if (!fallen) {
                notifyFall(totalStepsTraveled);
            }
        }

        if (requestReset) {
            if (aimode) {
                distData.push({'t':iterations,'dist':totalDistTraveled,'steps':totalStepsTraveled})
            }

            if (recordingSteps) {
                endRecording(false);
                console.log('(-) YOU DIED.')
            }

            step_phase = [false,false];
            stepBeginAngle = NaN;
            stepBackLeg = undefined;
            stepBackJoint = undefined;
            stepForwardLeg = undefined;
            lastStepX = 0;
            resetRunner();
            requestReset = false;
            respawning = true;
            deathCount++;
            scoreCheckpointDist = 2;
            scorePenaltyDist = 2;
            setTimeout(function() {
                respawning = false;
                activateWalkDelay();
            },1000)
        }
    }
}

function activateWalkDelay() {
    walkDelay = true;
    setTimeout(function() {
        walkDelay = false;
    },1000);
}

setInterval(mainLoop,1000*freq);
resetRunner();

function center_of_mass(body) {

    var xC=0,yC=0,totalM=0;
    for (var p in body) {
        var part = body[p];
        var m = part.GetMass();
        var pos = part.GetPosition();
        totalM += m;
        xC += (m*pos.x);
        yC += (m*pos.y);
    }
    xC /= totalM;
    yC /= totalM;
    return {'x':xC, 'y':yC};
}

function roundToTenThousandth(n) {
    return round(n * 10000) / 10000;
}

function roundToHundredth(n) {
    return round(n * 100) / 100;
}

function roundToTenth(n) {
    return round(n * 10) / 10;
}