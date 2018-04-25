surfc(ftt, Gaintt, FL_rms_err)
hold on

nminpts = 20;   %set number of min pts to collect
FL_rms_err_1col = FL_rms_err(:)
[tempmins,originalpos] = sort( FL_rms_err(:), 'ascend' );
mins = tempmins(1:nminpts)
pos=originalpos(1:nminpts)

for acnt = 1:nminpts
    indicCol(acnt,1) = idivide(int32(pos(acnt,1)), int32(sim_pts),'ceil')
    rem1 = mod(int32(pos(acnt,1)),int32(sim_pts))
    if(rem1 == 0); 
        indicRow(acnt,1) = sim_pts
    else
        indicRow(acnt,1) = rem1
    end
    hp1 = scatter3(ftt(indicCol(acnt,1)),Gaintt(indicRow(acnt,1)),FL_rms_err(indicRow(acnt,1),indicCol(acnt,1)),'filled');
    hp1.SizeData = 150;
end
xlabel('freq');
ylabel('Gain');