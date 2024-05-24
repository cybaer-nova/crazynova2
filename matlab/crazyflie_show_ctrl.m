% crazyflie load custom control log csv file

% read file
folder = "../../../";
csvfilename = "20240524T143724_CtrlLog.csv";
file = [folder csvfilename];
##T = readtable(csvfilename);


pkg load io; ## octave
data = csv2cell(file);
[Ndata,Nfields] = size(data)
for i=1:Nfields
  n1 = length(data{2,i})
  n2 = n1;
  if n1 > 3
    n2 = length(str2num(data{2,i}));
  endif
  eval([data{1,i} "=zeros(" num2str(Ndata-1) "," num2str(n2) ");"]);
  for j = 2:Ndata
    if n1 > 3
      code = [ data{1,i} "(" num2str(j-1) ",:)=" data{j,i} ";"];
    else
      code = [ data{1,i} "(" num2str(j-1) ",:)=" num2str(data{j,i}) ";"];
    end
    eval(code);
  end
end

% convert date to print format
t = ttag - ttag(1);
x = [pos';vel';lbd';om'];
x_ref = [pos_d';vel_d';lbd_d';om_d'];
u = u_acc';

% plot data
initPlots;
vehicle3d_ref_show_data(t,x,u,x_ref);