%Copyright, Wael Suleiman, 2016
function Plot_robot(P,fig_hold,Width)

if nargin()<2,
  fig_hold=false;
  Width=2;
end;

if nargin()<3,
  Width=2;
end;

if ~fig_hold,
  figure;
end;

hold on;
grid on;
xlabel('X (m)');
ylabel('Y (m)');

  for i=1:length(P),
          
	     X(i)=P(i).p(1);
         Y(i)=P(i).p(2);

end;
X=[0,X];
Y=[0,Y];
plot(X,Y,'-b','LineWidth',Width);
plot(X(1:end-1),Y(1:end-1), 'or','markersize', 12);
plot(X(end),Y(end), 'og','markersize', 12);
