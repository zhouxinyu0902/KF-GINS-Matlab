function corrected = applyErrorCorrection(navstate, dx, apply_attitude)
%APPLYERRORCORRECTION Apply the working EKF feedback sign convention.

    corrected = navstate;
    corrected.pos = corrected.pos - dx(1:3);
    corrected.vel = corrected.vel - dx(4:6);

    if apply_attitude
        qpn = rotvec2quat(dx(7:9));
        corrected.qbn = quatProd(qpn, corrected.qbn);
        corrected.qbn = quatNormalized(corrected.qbn);
        corrected.cbn = quat2dcm(corrected.qbn);
        corrected.att = dcm2euler(corrected.cbn);
    end

    corrected.gyrbias = corrected.gyrbias + dx(10:12);
    corrected.accbias = corrected.accbias + dx(13:15);

    param = Param();
    [corrected.Rm, corrected.Rn] = getRmRn(corrected.pos(1), param);
    corrected.gravity = getGravity(corrected.pos);
end
