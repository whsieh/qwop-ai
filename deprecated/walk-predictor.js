
/**
  Given: body, the Object containing each of the ragdoll's body parts; joint,
  the Object containing each of the ragdoll's joints; action, a number representing
  the current action of the ragdoll; and prevA, the previous action taken by the
  ragdoll, reconstructs a physics simulation and runs the simulation 100 ms. Returns
  the expected body, joint, environment, and world which results from running the
  simulation.

  If the head is below a threshold (i.e. 75) does not attempt to run the physics
  simulation. Often crashes when head, feet and/or torso are in contact with the
  ground simultaneously.
*/
function predictWorld(body,joint,action,prevA,timeMS) {

    timeMS = timeMS || 100

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

        _world.Step(0.001*timeMS,1,1);

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

function extractState(body,joint,environment,angles,prevAction) {
    if (!angles) {
        var lh = roundToTenThousandth(joint.l_hip.GetJointAngle());
        var rh = roundToTenThousandth(joint.r_hip.GetJointAngle());
        var lk = roundToTenThousandth(joint.l_knee.GetJointAngle());
        var rk = roundToTenThousandth(joint.r_knee.GetJointAngle());
    } else {
        var lh = angles.lh;
        var rh = angles.rh;
        var lk = angles.lk;
        var rk = angles.rk;
    }

    return {
        lh_ang: lh,
        rh_ang: rh,
        lk_ang: lk,
        rk_ang: rk,
        T_ang: roundToTenThousandth(body.torso.GetAngle()),
        LF_y: roundToTenth(getLeftFootY(body)),
        RF_y: roundToTenth(getRightFootY(body)),
        pA: prevAction,
        'body':body,
        'joint':joint,
        'env':environment
    };
}

function testPrediction() {
    var oldKey = keyState
    setTimeout(function() {
        var curKey = keyState
        if (curKey !== 0) {
            console.log('Testing state prediction, current key: ' + curKey)
            var pred = predictWorld(body,joint,curKey,oldKey,100)
            setTimeout(function() {
                if (keyState === curKey) {
                    console.log('Predicted: ')
                    console.log(extractState(pred.body,pred.joint,pred.env, pred.angles,curKey))
                    console.log('Actual')
                    console.log(extractState(body,joint,environment,null,curKey))
                } else {
                    console.log('Key selection changed, prediction not valid.')
                }
            },100)
        }
    },100)
}