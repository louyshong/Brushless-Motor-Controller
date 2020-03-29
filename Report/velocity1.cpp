    //update integral error
    acc_v_error += vError * 10;
    if(acc_v_error > MAX_V_ERROR) acc_v_error = MAX_V_ERROR;
    if(acc_v_error < -MAX_V_ERROR) acc_v_error = -MAX_V_ERROR;