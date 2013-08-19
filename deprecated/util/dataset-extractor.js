var curState, prevState, prevKeyState = 0;
var observing = true;
var sampleSize = 0;

$('body').append('<br>[');

function observeTransition() {
    if (observing) {
        curState = extractState();
        if (!respawning && keyState == prevKeyState && legalKeyStates[keyState] && prevState !== undefined) {
            sample = {};
            sample.s1 = prevState;
            sample.a = keyState;
            sample.s2 = curState;
            sample.r = (curVelX - body.head.GetPosition().y < 100 ? 2 : 0);
            $('body').append(JSON.stringify(sample) + ',');
            sampleSize ++;
            if (sampleSize % 100 == 0) {
                console.log('- gathered ' + sampleSize + ' data points...');
            }
        }
        prevKeyState = keyState;
        prevState = curState;
    }
    if (sampleSize >= 6000) {
        endObservation();
    }
}

function extractState() {
    return {
        lh_ang: roundToTenThousandth(joint.l_hip.GetJointAngle()),
        lh_w: roundToHundredth(joint.l_hip.GetJointSpeed()),
        lh_lim: hipAtLimit(joint.l_hip),

        rh_ang: roundToTenThousandth(joint.r_hip.GetJointAngle()),
        rh_w: roundToHundredth(joint.r_hip.GetJointSpeed()),
        rh_lim: hipAtLimit(joint.r_hip),

        lk_ang: roundToTenThousandth(joint.l_knee.GetJointAngle()),
        lk_w: roundToHundredth(joint.l_knee.GetJointSpeed()),
        lk_lim: kneeAtLimit(joint.l_knee),

        rk_ang: roundToTenThousandth(joint.r_knee.GetJointAngle()),
        rk_w: roundToHundredth(joint.r_knee.GetJointSpeed()),
        rk_lim: kneeAtLimit(joint.r_knee),

        T_ang: roundToTenThousandth(body.torso.GetAngle()),
        T_w: roundToHundredth(body.torso.GetAngularVelocity()),
        H_y: roundToTenth(body.head.GetPosition().y),
        H_vy: roundToHundredth(body.head.GetLinearVelocity().y),
        LLT_dx: roundToTenth(body.ll_leg.GetPosition().x - body.torso.GetPosition().x),
        LRT_dx: roundToTenth(body.lr_leg.GetPosition().x - body.torso.GetPosition().x),
        LF_y: roundToTenth(getLeftFootHeight()),
        RF_y: roundToTenth(getRightFootHeight()),
        LL_c: hasContact(body.ll_leg,environment.floor) ? 1 : 0,
        LR_c: hasContact(body.lr_leg,environment.floor) ? 1 : 0
    };
}

function beginObservation() {
    if (!observing) {
        observing = true;
    } else {
        setInterval(observeTransition, 100);
    }
}

function endObservation() {
    if (observing) {
        $('body').append('{\"total_size\": ' + sampleSize + '}]');
    }
    observing = false;
}

function roundToTenThousandth(n) {
    return Math.round(n * 10000) / 10000;
}

function roundToHundredth(n) {
    return Math.round(n * 100) / 100;
}

function roundToTenth(n) {
    return Math.round(n * 10) / 10;
}