
can(pickup(X), 	 [handempty, clear(X), on(X, Y)], robot).
can(stack(X, Y), [holding(X), clear(Y)], robot).
can(pickup(X), 	 [handempty, clear(X), ontable(X)], robot).
can(putdown(X),  [holding(X)], robot).

adds(pickup(X),  [clear(Y), holding(X)], _ , robot).
adds(stack(X,Y), [handempty, on(X,Y), clear(X)], _ , robot).
adds(pickup(X),  [holding(X)], _ , robot).
adds(putdown(X), [ontable(X), clear(X), handempty], _ , robot).

deletes(pickup(X),  [handempty, clear(X), on(X,Y)], robot).
deletes(stack(X,Y), [holding(X), clear(Y)], robot).
deletes(pickup(X),  [handempty, clear(X), ontable(X)], robot).
deletes(putdown(X), [holding(X)], robot).

