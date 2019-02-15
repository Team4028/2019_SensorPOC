package frc.robot.auton.pathfollowing.util;

import java.util.*;


public class Poset {
    int numVertices = 0;
    static List<node> nodesList = new ArrayList<node>();
    static List<edge> edgesList = new ArrayList<edge>();

    public Poset(int vertNum){
        numVertices = vertNum;
        for (int i = 0; i< vertNum; i++){
            nodesList.add(new node(i));
        }
    }

    public void addVertex(){
        numVertices++;
        nodesList.add(new node(numVertices-1));
    }

    public void removeVertex(node N){
        numVertices--;
        for (node c: N.getChildren()){
            for (node p: N.getParents()){
                removeEdge(new edge(N, c));
                removeEdge(new edge(p, N));
                addEdge(p, c);
            }
        }
        for (int ind = N.getIdentifier()+1; ind<numVertices; ind++){
            getNode(ind).changeIdentifier(getNode(ind).getIdentifier() -1);
        }
        nodesList.remove(N.getIdentifier());
        for (edge e: edgesList){
            e.update();
        }
    }

    public void removeEdge(edge e){
        edgesList.remove(e.getPosInEdgesList());
    }

    public void addEdge(node nf, node n2){
        edgesList.add(new edge(nf, n2));        
    }


    public node getNode(int nodeNum){
        try {
            return nodesList.get(nodeNum);
        } catch(Exception e){
            System.out.println("Error: Called Node Does Not Exist");
            return null;
        }
    }

    public boolean queryEdgeExistence(node maybeFrom, node maybeTo){
        for (edge e: edgesList){
            if (e.getIdentifiers() == new int[] {maybeFrom.getIdentifier(), maybeTo.getIdentifier()}){
                return true;
            }
        }
        return false;
    }

    public int[][] getSkewSymmetricLinearForm(){
        int[][] skewSymmetricLinearForm = new int[numVertices][numVertices];
        for (int i=0; i<numVertices; i++){
            for (int j=0; j<numVertices; j++){
                if (queryEdgeExistence(nodesList.get(i), nodesList.get(j))){
                    skewSymmetricLinearForm[i][j] = 1;
                } else if (queryEdgeExistence(nodesList.get(j), nodesList.get(i))) {
                    skewSymmetricLinearForm[i][j]=-1;
                } else {
                    skewSymmetricLinearForm[i][j] = 0;
                }
            }
        }
        return skewSymmetricLinearForm;
    }



    public static class node {
        int identifier;
        public node(int identifyingNum){
            identifier = identifyingNum;
        }

        public void changeIdentifier(int newIdentifier){
            identifier = newIdentifier;
        }

        public List<node> getParents(){
            List<node> parents = new ArrayList<node>();

            for (edge e : edgesList){
                if (e.getToIdentifier() == identifier){
                    parents.add(e.getFromNode());
                }
            }

            return parents;
        }

        public List<node> getChildren(){
            List<node> children = new ArrayList<node>();

            for (edge e : edgesList){
                if (e.getFromIdentifier() == identifier ){
                    children.add(e.getToNode());
                }
            }

            return children;
        }

        public int getIdentifier(){
            return identifier;
        }
        
    }

    public static class edge{
        int fromNodeIdentifier;
        int toNodeIdentifier;
        node fromNode;
        node toNode;
        
        public edge(node from, node to){
            fromNode = from;
            toNode = to;
            fromNodeIdentifier = fromNode.getIdentifier();
            toNodeIdentifier = toNode.getIdentifier();
        }

        public void update(){
            fromNodeIdentifier = fromNode.getIdentifier();
            toNodeIdentifier = toNode.getIdentifier();
        }

        public int getFromIdentifier(){
            return fromNodeIdentifier;
        }

        public int getToIdentifier(){
            return toNodeIdentifier;
        }

        public int[] getIdentifiers(){
            int[] ans = new int[] {fromNodeIdentifier, toNodeIdentifier};
            return ans;
        } 


        public node getFromNode() {
            return fromNode;
        }

        public node getToNode(){
            return toNode;
        }

        public node[] getNodes(){
            node[] ans = new node[] {fromNode, toNode};
            return ans;
        }

        public int getPosInEdgesList(){
            int guess = 0;
            boolean happy = false;
            while (! happy){
                happy = (edgesList.get(guess).getIdentifiers() == new int[] {fromNodeIdentifier, toNodeIdentifier});
                guess++;
            }
            return guess-1;
        }
    }


}