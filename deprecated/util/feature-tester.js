
function roundToTenThousandth(n) {
    return Math.round(n * 10000) / 10000;
}

function roundToHundredth(n) {
    return Math.round(n * 100) / 100;
}

function roundToTenth(n) {
    return Math.round(n * 10) / 10;
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

setInterval(function() {

    var s = extractState();

    var v1 = 0.00015*Math.pow((s.LLT_dx + s.LRT_dx)/2 + 10,2);
    var v2 = 2*Math.min(Math.max(Math.abs(s.T_ang + 0.2), 0.25) - 0.25, 0.5);
    var v3 = Math.pow(Math.max(0,Math.min(140,s.H_y))/140,3);
    var v4 = (s.LL_c + s.LR_c) / 2;
    var v5 = Math.max(s.LF_y>85||s.RF_y>85?1:0,Math.min(s.LF_y-45,s.RF_y-45)/50);

},250);