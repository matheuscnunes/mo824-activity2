import gurobi.*;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.Scanner;

public class KTsp extends GRBCallback {
    // Euclidean distance between points 'i' and 'j'
    protected static double distance(int[] x, int[] y, int i, int j) {
        double dx = x[i] - x[j];
        double dy = y[i] - y[j];
        return Math.sqrt(dx * dx + dy * dy);
    }

    public static void main(String[] args) {
        int[] vNum = new int[]{100, 150, 200, 250};
        int[] similaritiesValues = new int[]{0, 2, 1};

        int[] x1 = new int[250];
        int[] y1 = new int[250];
        int[] x2 = new int[250];
        int[] y2 = new int[250];

        String coordinatesFileName = "/home/matheus.nunes/Unicamp/2022/MO824-proj-teorico/ativ2/mo824-activity2/mo824_atividade2_coords.in";
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

                    // (e ∈ E)∑ De = k
                    GRBLinExpr dExpr = new GRBLinExpr();
                    for (int i = 0; i < v; i++) {
                        for (int j = 0; j < i; j++) {
                            dExpr.addTerm(1.0, shared[i][j]);
                        }
                    }
                    // TODO: maybe it could be GRB.GREATER_EQUAL, check if should be GRB.EQUAL because:
                    /*
                        "k = 0 resulta em uma solução factível ser qualquer par de ciclos Hamiltonianos."
                     */
                    model.addConstr(dExpr, GRB.EQUAL, k, "de_similarity");

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
    public KTsp(GRBVar[][] traveler1Vars, GRBVar[][] traveler2Vars) {
        this.traveler1Vars = traveler1Vars;
        this.traveler2Vars = traveler2Vars;
    }

    @Override
    protected void callback() {
        // TODO: Implement callback
    }
}
