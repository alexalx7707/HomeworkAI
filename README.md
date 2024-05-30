# HomeworkAI
 Travelling Salesman Problem Homework Artificial Intelligence Year 2 Semester 2

When the program runs it asks the user to press 1 if he wants an automatically generated input or 2 if he wants to do
it manually.
In case of the automatically generated graph, the user only has to input the number of vertices he wants and then the graph
is going to be printed in order for the user to see it, along with the responses for the 3 algorithms it uses to calculate the problem.
In the case of the manually generated input, the user has to input the number of vertices he wants, then he has to input
v*(v-1)/2 edges in order for the graph to be complete as such:
Example:
3
Bratislava Narnia 2999
Narnia Palilula 4500
Palilua Bratislava 1200

The format of the input needs to be
 
NoOfVertices
Vertice1 Vertice2 WeightOfEdge
etc.

The program uses 3 algorithms in order to solve the Travelling Salesman Problem, firstly the DFS alg, then UCS, then A*
All the algorithms output the minCost of the best path, the path itself and how long did it take for each one of them
to compile.