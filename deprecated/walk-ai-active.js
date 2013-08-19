
/* Q-learning rate - decrease over time */
var alpha = 0.95;

/* Discount factor - decrease for greedy behavior */
var gamma = 0.975;

/* Exploration constant - increase for more exploration priority */
var kappa = 15;

/* Discretization level of each continuous feature */
var RESOLUTION = 16;
var SQRTRES = sqrt(RESOLUTION);

/* Strength of preloaded 'walking' knowledge (i.e. how much
    do we want to cheat) */
var guidestrength = 64;

/* Used to determine if the AI has gotten itself stuck */
var stateHistory = [];

/* Used for finding nearest neighbors in the kd-walk tree */
var kd_near = 12;
var kd_max = 6;

/* Write scores to javascript console in real time if true */
var output = true;

/* The list of possible actions taken from any state */
var ACTIONS = [0,1,2,4,6,8,9];

/* Time between each reinforcement learning iteration */
var timePerActionMs = 100;
var actionsPerSec = 1000/timePerActionMs;

var features = [];
initFeatures();

var _Q = {};
var _N = {};
var FT_WEIGHTS = [1,1,1,1,3,2];

/* Data structures for storing prior walking knowledge */
var walkTree = new kdTree([],stateDist,features.length);
var walkActions = {};

/** 
  The Hamming distance between two state vectors. Used to build
  the k-d walk tree
*/
function stateDist(ka,kb) {
    var d = 0, l = features.length-1;
    // The last feature is a special case...
    for (var i = 0; i < l; i++) {
        d += FT_WEIGHTS[i]*abs(ka[i] - kb[i]);
    }
    // Take the individual 
    var u1=ka[l],u2=kb[l];
    d += FT_WEIGHTS[l]*(abs(floor(u1/SQRTRES)-floor(u2/SQRTRES))+
        abs((u1%SQRTRES)-(u2%SQRTRES)));
    return d;
}

/**
  Returns the numerical representation of a state as a vector of
  6 integers mod RESOLUTION
*/
function asVector(k) {
    var v = [,,,,,,],c=features.length-1;
    for (var i = 0; i < features.length; i++) {
        v[c] = floor(k/pow(RESOLUTION,i))%RESOLUTION;
        c--;
    }
    return v;
}

/**
  Compresses a vector of 6 integers mod RESOLUTION as a single integer
*/
function asNumber(v) {
    var k = 0;
    v.forEach(function(i) {
        k = (RESOLUTION*k) + i
    })
    return k;
}

/**
  Initializes all of the features.
*/
function initFeatures() {

    features.push(function(s1,a) {                      // v0 left knee
        var s = (a==undefined) ? s1 : transition(s1,a);
        var val = 0.8*(s.lk_ang + 0.25);
        return val;
    });

    features.push(function(s1,a) {                      // v1 right knee
        var s = (a==undefined) ? s1 : transition(s1,a);
        var val = 0.8*(s.rk_ang + 0.25);
        return val;
    });

    features.push(function(s1,a) {                      // v2 left hip
        var s = (a==undefined) ? s1 : transition(s1,a);
        var val = 0.5*(s.lh_ang + 1);
        return val;
    });

    features.push(function(s1,a) {                      // v3 right hip
        var s = (a==undefined) ? s1 : transition(s1,a);
        var val = 0.5*(s.rh_ang + 1);
        return val;
    });

    features.push(function(s1,a) {                      // v4 torso ang
        var s = (a==undefined) ? s1 : transition(s1,a);
        var val = 0.4*(1.25+max(-1.25,min(1.25,s.T_ang)));
        return val;
    });

    features.push(function(s1,a) {                      // v5 feet heights
        var s = (a==undefined) ? s1 : transition(s1,a);
        var val = min(max(floor(s.LF_y/20),0),SQRTRES-1);
        val = (val*SQRTRES) + min(max(floor(s.RF_y/20),0),SQRTRES-1);
        return val/RESOLUTION;
    });
}

function approxQ(s,a) {
    var k = encodeState(s);
    var data = searchQTree(k,a)
    if (k in _Q && a in _Q[k]) {
        data.push([_Q[k][a],0]);
    }
    if (data.length > 0) {
        var totalWt = 0, total = 0;
        data.forEach(function(p) {
            var wt = 1/(0.5+pow(p[1],2));
            totalWt += wt;
            total += wt*p[0];
        });
        return total/totalWt;
    }
    return 0;
}

function Q(s,a) {
    var k = encodeState(s);
    if (k in _Q && a in _Q[k]) {
        return _Q[k][a];
    }
    return 0;
}

function N(s,a) {
    var k = encodeState(s);
    if (k in _N && a in _N[k]) {
        return _N[k][a] + 1;
    }
    return 1;
}

function update(s1,a,r,s2) {
    var maxQ = -Infinity;
    ACTIONS.forEach(function(a2) {
        var q = Q(s2,a2);
        if (q > maxQ) {
            maxQ = q;
        }
    });
    var qvalue = (r + gamma*maxQ)
    qvalue = (alpha*qvalue) + (1-alpha)*Q(s1,a);
    updateQ(s1,a,qvalue);
}

function updateQ(s,a,value) {
    if (value !== 0) {
        var k = encodeState(s);
        if (!(k in _Q)) {
            _Q[k] = {};
        }
        // if (!(a in _Q[k])) {
        //     updateQTree(k,a);
        // }
        _Q[k][a] = round(1000*value)/1000;
    }
}

function updateN(s,a) {
    var k = encodeState(s);
    if (k in _N) {
        if (a in _N[k]) {
            _N[k][a]++;
        } else {
            _N[k][a] = 1;
        }
    } else {
        _N[k] = {};
        _N[k][a] = 1;
    }
}

function encodeState(s) {
    if ('code' in s) {
        return s.code;
    } else {
        var total = 0;
        features.forEach(function(f) {
            var x = discretize(f(s),RESOLUTION);
            total = (RESOLUTION*total) + x;
        });
        s.code = total;
        return total;
    }
}

function discretize(v,n,lb,ub) {
    lb = (lb==undefined) ? 0 : lb;
    ub = (ub==undefined) ? 1 : ub;
    var r = min(n-1,max(0,floor((v-lb)*n)));
    return r;
}

function explore(s,a) {
    return Q(s,a) + (kappa/N(s,a));
}

function getActionBias(s) {

    var bias = {0:0,1:0,2:0,4:0,6:0,8:0,9:0};
    if (!walkDelay) {
        var k = encodeState(s);
        var vnear = walkTree.nearest(asVector(k),kd_max,kd_near);
        // if (vnear.length > 0)
        //     console.log('[ Found: ' + vnear.length + ' results ]')
        vnear.forEach(function(vdata) {
            var a = walkActions[asNumber(vdata[0])];
            bias[a] += guidestrength/pow(0.5+vdata[1],2);
        });
    }
    return bias;
}

function chooseAction(s) {
    var maxF = -Infinity;
    var best = [];
    // var actionbias = getActionBias(s);
    ACTIONS.forEach(function(a) {
        // var f = explore(s,a) + actionbias[a];
        var f = Q(s,a)
        if (f > maxF) {
            maxF = f;
            best = [a];
        } else if (f == maxF) {
            best.push(a);
        }
    });
    return best[floor(best.length*random())];
}

/**
  Attempts to predict the state 100 ms later.
  Note: transition(s1,a) now caches the new state
  s2 as the attribute a in s1, for future use.
*/
function transition(s1,a) {
    if (a in s1) {
        return s1[a];
    } else {
        var info = predictWorld(s1['body'], s1['joint'],a,s1['pA']);
        var s2 = extractState(info['body'], info['joint'], info['env'],info['angles'],a);

        var lk_ang = s2['lk_ang'];
        var rk_ang = s2['rk_ang'];
        var lh_ang = s2['lh_ang'];
        var rh_ang = s2['rh_ang'];

        s2['lk_ang'] = lk_ang > kneeLimits[1] ? kneeLimits[1] : lk_ang;
        s2['lk_ang'] = lk_ang < kneeLimits[0] ? kneeLimits[0] : lk_ang;
        s2['rk_ang'] = rk_ang > kneeLimits[1] ? kneeLimits[1] : rk_ang;
        s2['rk_ang'] = rk_ang < kneeLimits[0] ? kneeLimits[0] : rk_ang;

        s2['lh_ang'] = lh_ang > hipLimits[1] ? hipLimits[1] : lh_ang;
        s2['lh_ang'] = lh_ang < hipLimits[0] ? hipLimits[0] : lh_ang;
        s2['rh_ang'] = rh_ang > hipLimits[1] ? hipLimits[1] : rh_ang;
        s2['rh_ang'] = rh_ang < hipLimits[0] ? hipLimits[0] : rh_ang;
        
        s1[a] = s2;
        return s2;
    }
}

function predictWorld(body,joint,action,prevA) {

    prevA = (prevA == undefined) ? 0 : prevA;
    action = (action == undefined) ? 0 : action;

    var headPos = body.head.GetPosition();

    if (headPos.y > 75) {

        var _world = new b2World(new b2Vec2(0, -75), true);
        var _environment = initWalls(_world,500,500,24);

        var _body = {};
        var _joint = {};

        var ul_legPos = body.ul_leg.GetPosition();
        var ur_legPos = body.ur_leg.GetPosition();
        var ll_legPos = body.ll_leg.GetPosition();
        var lr_legPos = body.lr_leg.GetPosition();
        var torsoPos = body.torso.GetPosition();

        var headVel = body.head.GetLinearVelocity();
        var ul_legVel = body.ul_leg.GetLinearVelocity();
        var ur_legVel = body.ur_leg.GetLinearVelocity();
        var ll_legVel = body.ll_leg.GetLinearVelocity();
        var lr_legVel = body.lr_leg.GetLinearVelocity();
        var torsoVel = body.torso.GetLinearVelocity();

        var ul_legAngVel = body.ul_leg.GetAngularVelocity();
        var ur_legAngVel = body.ur_leg.GetAngularVelocity();
        var ll_legAngVel = body.ll_leg.GetAngularVelocity();
        var lr_legAngVel = body.lr_leg.GetAngularVelocity();
        var torsoAngVel = body.torso.GetAngularVelocity();

        var ul_legAng = body.ul_leg.GetAngle();
        var ur_legAng = body.ur_leg.GetAngle();
        var ll_legAng = body.ll_leg.GetAngle();
        var lr_legAng = body.lr_leg.GetAngle();
        var torsoAng = body.torso.GetAngle();

        var _head = createBall(_world,headPos.x,headPos.y,25,false,0.1);
        _head.SetLinearVelocity(headVel);

        var _ul_leg = createBox(_world,ul_legPos.x-5,ul_legPos.y-20,10,40,PI/6,false,10);
        _ul_leg.SetLinearVelocity(ul_legVel);
        _ul_leg.SetAngularVelocity(ul_legAngVel);
        _ul_leg.SetAngle(ul_legAng);

        var _ll_leg = createBox(_world,ll_legPos.x-5,ll_legPos.y-20,10,40,-PI/6,false,10);
        _ll_leg.SetLinearVelocity(ll_legVel);
        _ll_leg.SetAngularVelocity(ll_legAngVel);
        _ll_leg.SetAngle(ll_legAng);

        var _torso = createBox(_world,torsoPos.x-16,torsoPos.y-35,32,70,0,false,5);
        _torso.SetLinearVelocity(torsoVel);
        _torso.SetAngularVelocity(torsoAngVel);
        _torso.SetAngle(torsoAng);

        var _ur_leg = createBox(_world,ur_legPos.x-5,ur_legPos.y-20,10,40,PI/6,false,10);
        _ur_leg.SetLinearVelocity(ur_legVel);
        _ur_leg.SetAngularVelocity(ur_legAngVel);
        _ur_leg.SetAngle(ur_legAng);

        var _lr_leg = createBox(_world,lr_legPos.x-5,lr_legPos.y-20,10,40,-PI/6,false,10);
        _lr_leg.SetLinearVelocity(lr_legVel);
        _lr_leg.SetAngularVelocity(lr_legAngVel);
        _lr_leg.SetAngle(lr_legAng);

        _body.head = _head;
        _body.torso = _torso;
        _body.ll_leg = _ll_leg;
        _body.ul_leg = _ul_leg;
        _body.lr_leg = _lr_leg;
        _body.ur_leg = _ur_leg;
        _head.SetUserData('head');
        _torso.SetUserData('torso');
        _ll_leg.SetUserData('ll_leg');
        _ul_leg.SetUserData('ul_leg');
        _lr_leg.SetUserData('lr_leg');
        _ur_leg.SetUserData('ur_leg');

        // Connect head and torso
        var _neck_jointDef = new b2WeldJointDef();
        var _neck_anchor = joint.neck.GetAnchorB();
        _neck_jointDef.Initialize(_head, _torso, _neck_anchor);
        var _neck_joint = _world.CreateJoint(_neck_jointDef);

        // Connect upper, lower left leg
        var _l_knee_jointDef = new b2RevoluteJointDef();
        var _l_knee_anchor = joint.l_knee.GetAnchorB();
        _l_knee_jointDef.Initialize(_ul_leg, _ll_leg, _l_knee_anchor);
        var _l_knee_joint = _world.CreateJoint(_l_knee_jointDef);
        _l_knee_joint.SetMotorSpeed(joint.l_knee.GetMotorSpeed());

        // Connect upper, lower right leg
        var _r_knee_jointDef = new b2RevoluteJointDef();
        var _r_knee_anchor = joint.r_knee.GetAnchorB();
        _r_knee_jointDef.Initialize(_ur_leg, _lr_leg, _r_knee_anchor);
        var _r_knee_joint = _world.CreateJoint(_r_knee_jointDef);
        _r_knee_joint.SetMotorSpeed(joint.r_knee.GetMotorSpeed());

        // Attach left, right legs to torso
        var _l_hip_jointDef = new b2RevoluteJointDef();
        var _l_hip_anchor = joint.l_hip.GetAnchorB();
        _l_hip_jointDef.Initialize(_torso, _ul_leg, _l_hip_anchor);
        var _l_hip_joint = _world.CreateJoint(_l_hip_jointDef);
        _l_hip_joint.SetMotorSpeed(joint.l_hip.GetMotorSpeed());

        var _r_hip_jointDef = new b2RevoluteJointDef();
        var _r_hip_anchor = joint.r_hip.GetAnchorB();
        _r_hip_jointDef.Initialize(_torso, _ur_leg, _r_hip_anchor);
        var _r_hip_joint = _world.CreateJoint(_r_hip_jointDef);
        _r_hip_joint.SetMotorSpeed(joint.r_hip.GetMotorSpeed());

        _joint.neck = _neck_joint;
        _joint.l_hip = _l_hip_joint;
        _joint.r_hip = _r_hip_joint;
        _joint.l_knee = _l_knee_joint;
        _joint.r_knee = _r_knee_joint;

        setFilterGroup([_torso,_ul_leg,_ur_leg,_ll_leg,_lr_leg],-2);

        lockRevoluteJoint(_l_knee_joint);
        lockRevoluteJoint(_r_knee_joint);
        lockRevoluteJoint(_l_hip_joint);
        lockRevoluteJoint(_r_hip_joint);

        _l_hip_joint.EnableLimit(true);
        _l_hip_joint.SetLimits(hipLimits[0],hipLimits[1]);   // -1 and 1
        _r_hip_joint.EnableLimit(true);
        _r_hip_joint.SetLimits(hipLimits[0],hipLimits[1]);
        _l_knee_joint.EnableLimit(true);
        _l_knee_joint.SetLimits(kneeLimits[0],kneeLimits[1]);    // -0.25 and 1
        _r_knee_joint.EnableLimit(true);
        _r_knee_joint.SetLimits(kneeLimits[0],kneeLimits[1]);

        var m = 1;
        for (var i = 0; i < 4; i++) {
            var delta = (action&m)-(prevA&m);
            if (delta == -m) {
                if (m == 1 || m == 2) {
                    _l_hip_joint.SetMotorSpeed(0);
                    _r_hip_joint.SetMotorSpeed(0);
                } else if (m == 4 || m == 8) {
                    _l_knee_joint.SetMotorSpeed(0);
                    _r_knee_joint.SetMotorSpeed(0);
                }
            }
            m *= 2;
        }

        switch(action) {

            case 0:
            break;

            case 1:
            _joint.l_hip.SetMotorSpeed(l_hip_rotate_speed);
            _joint.r_hip.SetMotorSpeed(-r_hip_rotate_speed);
            break;

            case 2:
            _joint.l_hip.SetMotorSpeed(-l_hip_rotate_speed);
            _joint.r_hip.SetMotorSpeed(r_hip_rotate_speed);
            break;

            case 4:
            _joint.l_knee.SetMotorSpeed(l_knee_rotate_speed);
            _joint.r_knee.SetMotorSpeed(-r_knee_rotate_speed);
            break;

            case 6:
            _joint.l_hip.SetMotorSpeed(-l_hip_rotate_speed);
            _joint.r_hip.SetMotorSpeed(r_hip_rotate_speed);
            _joint.l_knee.SetMotorSpeed(l_knee_rotate_speed);
            _joint.r_knee.SetMotorSpeed(-r_knee_rotate_speed);
            break;

            case 8:
            _joint.r_knee.SetMotorSpeed(r_knee_rotate_speed);
            _joint.l_knee.SetMotorSpeed(-l_knee_rotate_speed);
            break;


            case 9:
            _joint.l_hip.SetMotorSpeed(l_hip_rotate_speed);
            _joint.r_hip.SetMotorSpeed(-r_hip_rotate_speed);
            _joint.r_knee.SetMotorSpeed(r_knee_rotate_speed);
            _joint.l_knee.SetMotorSpeed(-l_knee_rotate_speed);
            break;
        }

        _world.Step(0.001*timePerActionMs,2,2);

        var angles = {
            'lk':_joint.l_knee.GetJointAngle() + joint.l_knee.GetJointAngle(),
            'rk':_joint.r_knee.GetJointAngle() + joint.r_knee.GetJointAngle(),
            'lh':_joint.l_hip.GetJointAngle() + joint.l_hip.GetJointAngle(),
            'rh':_joint.r_hip.GetJointAngle() + joint.r_hip.GetJointAngle()
        }

        return {'body':_body, 'joint':_joint, 'env':_environment, 'world':_world, 'angles':angles};

    } else {
        return {'body':body, 'joint':joint, 'env':environment, 'world':world};

    }
}
