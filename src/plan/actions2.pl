block(a).
block(b).
block(c).
robot(nao1).
position(room1).
position(room2).
support(plateau).

% take
can(take(R, X, At), [handempty(R), clear(X), at(R,At), at(X,At)], test2) :- 
				block(X), robot(R), position(At).
adds(take(R, X, At), [holding(R,X)], _ , test2) :- 
			block(X), robot(R).
deletes(take(R, X, At), [handempty(R), clear(X)], test2) :- 
			block(X), robot(R).

% go
can(go(R, From, To), [handempty(R), at(R, From)], test2) :-
				position(From), position(To), robot(R), From \= To.
adds(go(R, From, To), [at(R, To)], _ , test2) :- 
			position(To), robot(R).
deletes(go(R, From, To), [at(R,From)], test2) :- 
			position(From), robot(R).

% puton
can(put(R, X, On), [holding(R,X), at(R, At), at(On, At), free(On)], test2) :-
			block(X), robot(R), position(At), support(On).
adds(put(R, X, On), [on(X, On), handempty(R), clear(X)], _ , test2) :- 
			support(On), robot(R), block(X).
deletes(put(R, X, On), [holding(R,X), free(On)], test2) :- 
			block(X), robot(R), support(On).



% et ca pour tester direct en PROLOG
test(P) :- plan([handempty(nao1), clear(a), at(nao1, room1), at(a, room2), free(plateau), at(plateau, room2)], [handempty(nao1), on(a,plateau)], test2, P).
