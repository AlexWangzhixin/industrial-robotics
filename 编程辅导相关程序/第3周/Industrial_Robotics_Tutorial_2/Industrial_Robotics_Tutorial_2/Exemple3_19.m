% solution of example 3.19 of Lynch
L = 1;
Tdb = [0   0  -1  250/L;
       0  -1   0 -150/L;
      -1   0   0  200/L;
       0   0   0    1];
   
Tde = [0   0  -1  300/L;
       0  -1   0  100/L;
      -1   0   0  120/L;
       0   0   0   1];
   
Tad = [0  0  -1  400/L;
       0  -1  0   50/L;
       -1  0  0  300/L;
       0   0  0    1];

a = 1/sqrt(2);
Tbc = [0 -a  -a  30/L;
       0  a  -a  -40/L;
       1  0   0   25/L;
       0  0   0    1];
   
   
Tbd = TransInv(Tdb);
Ted = TransInv(Tde);
Tda=TransInv(Tad);
Tcb=TransInv(Tbc);
Tab = Tad*Tdb;
Tac = Tab*Tbc;
Tae = Tad*Tde;
l = 50;
figure(1)
plotvol([400  300 300  ])
trplot(eye(4),'framelabel', 'A','length', l)
trplot(Tad,'framelabel', 'D',  'length', l)
trplot(Tab,'framelabel', 'B' , 'length', l)
trplot(Tac,'framelabel', 'C'  ,'length', l)
trplot(Tae,'color','k', 'framelabel', 'E', 'length', l)

Tce = TransInv(Tad*Tdb*Tbc)*Tad*Tde;
Tae2 = Tac*Tce; 
trplot(Tae2, 'color','r', 'length', l)
Tce2 = Tcb*Tbd*Tde;
trplot(Tac*Tce2, 'color','g', 'length', l)
