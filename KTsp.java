import gurobi.*;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.Scanner;

public class KTsp extends GRBCallback {
    private static int TIME_LIMIT_IN_SECONDS = 30 * 60; // 30 minutes
    // Euclidean distance between points 'i' and 'j'
    protected static int distance(int[] x, int[] y, int i, int j) {
        int dx = x[i] - x[j];
        int dy = y[i] - y[j];
        return (int) Math.ceil(Math.sqrt(dx * dx + dy * dy));
    }

    public static void main(String[] args) {
        int[] vNum = new int[]{100, 150, 200, 250};
        int[] similaritiesValues = new int[]{0, 2, 1};

        int[] x1 = new int[250];
        int[] y1 = new int[250];
        int[] x2 = new int[250];
        int[] y2 = new int[250];

        String coordinatesFileName = args[0];
        try (FileInputStream fis = new FileInputStream(coordinatesFileName);
            Scanner sc = new Scanner(fis)) {
            int i = 0;
            while (sc.hasNextLine()) {
                String[] parts = sc.nextLine().split("\\s");
                x1[i] = Integer.parseInt(parts[0]);
                y1[i] = Integer.parseInt(parts[1]);
                x2[i] = Integer.parseInt(parts[2]);
                y2[i] = Integer.parseInt(parts[3]);

                i++;
            }
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }

        for (Integer v : vNum) {
            for (Integer similarity : similaritiesValues) {
                Integer k = similarity == 0 ? 0 : (v / similarity);

                System.out.println("\n\n######################## START ########################");
                System.out.println("Executing kTSP for |V|=" + v + " and similarity k=" + k);

                try {
                    GRBEnv env = new GRBEnv();
                    GRBModel model = new GRBModel(env);

                    // Must set LazyConstraints parameter when using lazy constraints
                    model.set(GRB.IntParam.LazyConstraints, 1);

                    // Create variables
                    GRBVar[][] traveler1 = new GRBVar[v][v];
                    GRBVar[][] traveler2 = new GRBVar[v][v];
                    GRBVar[][] shared = new GRBVar[v][v];

                    for (int i = 0; i < v; i++) {
                        for (int j = 0; j <= i; j++) {
                            // x1
                            traveler1[i][j] = model.addVar(0.0, 1.0, distance(x1, y1, i, j),
                                    GRB.BINARY, "t1_[" + i + "][" + j + "]");
                            traveler1[j][i] = traveler1[i][j];

                            // x2
                            traveler2[i][j] = model.addVar(0.0, 1.0, distance(x2, y2, i, j),
                                    GRB.BINARY, "t2_[" + i + "][" + j + "]");
                            traveler2[j][i] = traveler1[i][j];

                            // De
                            shared[i][j] = model.addVar(0.0, 1.0, 0.0,
                                    GRB.BINARY, "d[" + i + "][" + j + "]");
                            shared[j][i] = shared[i][j];
                        }
                    }

                    // (e ∈ sigma(i))∑ x1 = 2 ∀ i V
                    // (e ∈ sigma(i))∑ x2 = 2 ∀ i V
                    for (int i = 0; i < v; i++) {
                        GRBLinExpr t1_expr = new GRBLinExpr();
                        GRBLinExpr t2_expr = new GRBLinExpr();
                        for (int j = 0; j < v; j++) {
                            t1_expr.addTerm(1.0, traveler1[i][j]);
                            t2_expr.addTerm(1.0, traveler2[i][j]);
                        }
                        model.addConstr(t1_expr, GRB.EQUAL, 2.0, "t1_deg2_" + i);
                        model.addConstr(t2_expr, GRB.EQUAL, 2.0, "t2_deg2_" + i);
                    }

                    // xe1 + xe2 = 2De -> xe1 + xe2 - 2De = 0
                    for (int i = 0; i < v; i++) {
                        for (int j = 0; j < i; j++) {
                            GRBLinExpr expr = new GRBLinExpr();
                            expr.addTerm(1.0, traveler1[i][j]);
                            expr.addTerm(1.0, traveler2[i][j]);
                            expr.addTerm(-2.0, shared[i][j]);
                            model.addConstr(expr, GRB.EQUAL, 0, "x1_plus_x2_equals_de2[" + i + "][" + j + "]");
                        }
                    }

                    // (e ∈ E)∑ De = k
                    GRBLinExpr dExpr = new GRBLinExpr();
                    for (int i = 0; i < v; i++) {
                        for (int j = 0; j < i; j++) {
                            dExpr.addTerm(1.0, shared[i][j]);
                        }
                    }
                    model.addConstr(dExpr, GRB.GREATER_EQUAL, k, "de_similarity");
                    // TODO: Check why GRB.EQUAL is infeasible when k < |V|
                    // Forbid edge from node back to itself
                    for (int i = 0; i < v; i++) {
                        traveler1[i][i].set(GRB.DoubleAttr.UB, 0.0);
                        traveler2[i][i].set(GRB.DoubleAttr.UB, 0.0);
                        shared[i][i].set(GRB.DoubleAttr.UB, 0.0);
                    }

                    model.setCallback(new KTsp(traveler1, traveler2, shared));
                    model.set(GRB.DoubleParam.TimeLimit, TIME_LIMIT_IN_SECONDS);
                    // TODO: Add rest of restrictions + callback to optimize
                     model.optimize();

                    if (model.get(GRB.IntAttr.SolCount) > 0) {
                        int[] t1Tour = findsubtour(model.get(GRB.DoubleAttr.X, traveler1));
                        int[] t2Tour = findsubtour(model.get(GRB.DoubleAttr.X, traveler2));
                        assert t1Tour.length == v;
                        assert t2Tour.length == v;

                        double[][] sharedTour = model.get(GRB.DoubleAttr.X, shared);
                        int sharedCount = 0;
                        for (int i = 0; i < v; i++) {
                            for (int j = 0; j < i; j++) {
                                sharedCount += sharedTour[i][j];
                            }
                        }
                        System.out.println("Shared tour betwen T1 and T2: " + sharedCount);

                        System.out.print("Tour T1 (" + t1Tour.length + "): ");
                        for (int i = 0; i < t1Tour.length; i++) {
                            System.out.print(t1Tour[i] + " ");
                        }
                        System.out.println();

                        System.out.print("Tour T2 (" + t2Tour.length + "): ");
                        for (int i = 0; i < t2Tour.length; i++) {
                            System.out.print(t2Tour[i] + " ");
                        }
                        System.out.println();
                    }
                    System.out.println("######################## END ########################\n\n");

                    // Dispose of model and environment
                    model.dispose();
                    env.dispose();

                } catch (GRBException e) {
                    System.out.println("Error code: " + e.getErrorCode() + ". " +
                            e.getMessage());
                    e.printStackTrace();
                }
            }
        }
    }

    private GRBVar[][] traveler1Vars;
    private GRBVar[][] traveler2Vars;
    private GRBVar[][] shared;
    public KTsp(GRBVar[][] traveler1Vars, GRBVar[][] traveler2Vars, GRBVar[][] shared) {
        this.traveler1Vars = traveler1Vars;
        this.traveler2Vars = traveler2Vars;
        this.shared = shared;
    }

    // Given an integer-feasible solution 'sol', return the smallest
    // sub-tour (as a list of node indices).
    protected static int[] findsubtour(double[][] sol) {
        int n = sol.length;
        boolean[] seen = new boolean[n];
        int[] tour = new int[n];
        int bestInd, bestLen;
        int i, node, len, start;

        for (i = 0; i < n; i++) {
            seen[i] = false;
        }

        start = 0;
        bestLen = n + 1;
        bestInd = -1;
        while (start < n) {
            // Get unseen node
            for (node = 0; node < n; node++) {
                if (!seen[node]) {
                    break;
                }
            }
            if (node == n) {
                break;
            }

            for (len = 0; len < n; len++) {
                tour[start + len] = node;
                seen[node] = true;
                for (i = 0; i < n; i++) {
                    if (sol[node][i] > 0.5 && !seen[i]) {
                        node = i;
                        break;
                    }
                }
                if (i == n) {
                    len++;
                    if (len < bestLen) {
                        bestLen = len;
                        bestInd = start;
                    }
                    start += len;
                    break;
                }
            }
        }

        int result[] = new int[bestLen];
        for (i = 0; i < bestLen; i++) {
            result[i] = tour[bestInd + i];
        }
        return result;
    }

    @Override
    protected void callback() {
        try {
            if (where == GRB.CB_MIPSOL) {
                // Found an integer feasible solution - does it visit every node?
                int n = traveler1Vars.length;
                int[] tour = findsubtour(getSolution(traveler1Vars));

                if (tour.length < n) {
                    // Add subtour elimination constraint
                    GRBLinExpr expr = new GRBLinExpr();
                    for (int i = 0; i < tour.length; i++)
                        for (int j = i + 1; j < tour.length; j++)
                            expr.addTerm(1.0, traveler1Vars[tour[i]][tour[j]]);
                    addLazy(expr, GRB.LESS_EQUAL, tour.length-1);
                }

                n = traveler2Vars.length;
                tour = findsubtour(getSolution(traveler2Vars));

                if (tour.length < n) {
                    // Add subtour elimination constraint
                    GRBLinExpr expr = new GRBLinExpr();
                    for (int i = 0; i < tour.length; i++)
                        for (int j = i+1; j < tour.length; j++)
                            expr.addTerm(1.0, traveler2Vars[tour[i]][tour[j]]);
                    addLazy(expr, GRB.LESS_EQUAL, tour.length - 1);
                }
            }
        } catch (GRBException e) {
            System.out.println("Error code: " + e.getErrorCode() + ". " +
                    e.getMessage());
            e.printStackTrace();
        }
    }
}
