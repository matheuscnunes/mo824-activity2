import java.io.FileInputStream;
import java.io.IOException;
import java.util.Scanner;

public class KTsp {
    // Euclidean distance between points 'i' and 'j'
    protected static double distance(double[] x, double[] y, int i, int j) {
        double dx = x[i] - x[j];
        double dy = y[i] - y[j];
        return Math.sqrt(dx * dx + dy * dy);
    }

    public static void main(String[] args) {
        Integer[] vNum = new Integer[]{100, 150, 200, 250};
        Integer[] similaritiesValues = new Integer[]{0, 2, 1};

        Integer[] x1 = new Integer[250];
        Integer[] y1 = new Integer[250];
        Integer[] x2 = new Integer[250];
        Integer[] y2 = new Integer[250];

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

            }
        }
    }
}
