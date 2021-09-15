function shutdown_sim_cb(~, msg, ~)

    global ShutdownSim;
    
    ShutdownSim = msg.Data;
end