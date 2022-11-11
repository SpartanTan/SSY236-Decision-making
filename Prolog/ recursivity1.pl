:- discontiguous father_of/2.
start:-write('find ancestor of A'), nl, write('ancestor(A,X)').
father_of(a,b).
father_of(a,c).
father_ok(c,d).
father_of(d,e).
father_of(d,f).
father_of(n,w).
father_of(w,y).
father_of(z,y).
father_of(y,x).
father_of(y,m).
father_of(x,f).
father_of(x,t).

%ancestor(A, B):- father_of(A,B).
%ancestor(A, B):- father_of(A,Z), ancestor(Z,B). % Z=y

ancestor(A, B, 1):- father_of(A,B).
ancestor(A ,B, N):- father_of(A,Z), K is N-1, ancestor(Z,B,K). % Z=y

nat(X):- integer(X), X>0.
/** <examples>
?- father_of(X,d)
*/
