package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ImprovedSkystoneDetector extends DogeCVDetector {
    public static double START_ROW_FRAC = 0.65;
    public static double END_ROW_FRAC = 0.9;
    public static double START_COL_FRAC = 0.28;
    public static double END_COL_FRAC = 0.88;


    public static int GRAYSCALE_FILTER_LOWER = 0;
    public static int GRAYSCALE_FILTER_UPPER = 40;

    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Setting to decide to use MaxAreaScorer or PerfectAreaScorer

    //Create the default filters and scorers
    public DogeCVColorFilter blackFilter = new GrayscaleFilter(GRAYSCALE_FILTER_LOWER, GRAYSCALE_FILTER_UPPER);
    public DogeCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 70); //Default Yellow blackFilter

    public RatioScorer ratioScorer = new RatioScorer(1.25, 3); // Used to find the short face of the stone
    public MaxAreaScorer maxAreaScorer = new MaxAreaScorer(0.01);                    // Used to find largest objects
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(5000, 0.05); // Used to find objects near a tuned area value


    // Results of the detector
    private Point screenPosition = new Point(); // Center screen position of the block
    private Rect foundRect = new Rect(); // Found rect

    private Mat rawImage = new Mat();
    private Mat workingMat = new Mat();
    private Mat displayMat = new Mat();
    private Mat blackMask = new Mat();
    private Mat hierarchy = new Mat();

    public Point getScreenPosition() {
        return screenPosition;
    }

    public Rect foundRectangle() {
        return foundRect;
    }


    @Override
    public Mat process(Mat input) {
        // These need to be computed at program start to prevent FTC dashboard from changing the values
        // of START_ROW_FRAC and the like between when we crop RawImage and when we copy it and output it
        int startRow = (int) (input.rows() * START_ROW_FRAC);
        int endRow = (int) (input.rows() * END_ROW_FRAC);
        int startCol = (int) (input.cols() * START_COL_FRAC);
        int endCol = (int) (input.cols() * END_COL_FRAC);

        rawImage = input.submat(startRow, endRow, startCol, endCol);
        rawImage.copyTo(workingMat);
        rawImage.copyTo(displayMat);
        rawImage.copyTo(blackMask);

        // Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);

        blackFilter.process(workingMat.clone(), blackMask);
        List<MatOfPoint> contoursBlack = new ArrayList<>();

        Imgproc.findContours(blackMask, contoursBlack, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat, contoursBlack, -1, new Scalar(40, 40, 40), 2);

        // Current result
        Rect bestRect = foundRect;
        double bestDifference = Double.MAX_VALUE; // MAX_VALUE since less difference = better

        for (MatOfPoint cont : contoursBlack) {
            double score = calculateScore(cont); // Get the difference score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0, 0, 255), 2); // Draw rect

            // If the result is better then the previously tracked one, set this rect as the new best
            if (score < bestDifference) {
                bestDifference = score;
                bestRect = rect;
            }
        }
        Imgproc.cvtColor(blackMask, blackMask, Imgproc.COLOR_GRAY2BGR);

        Mat outMat = new Mat();
        input.copyTo(outMat);
        Point trueTL = new Point(startCol, startRow);
        Point trueBR = new Point(endCol, endRow);
        Rect crop = new Rect(trueTL, trueBR);


        if (bestRect != null) {
            // Show chosen result
            Imgproc.rectangle(outMat, crop.tl(), crop.br(), new Scalar(0, 0, 255), 4);
            Rect subRect = new Rect(new Point(startCol + bestRect.tl().x, startRow + bestRect.tl().y),
                    new Point(startCol + bestRect.br().x, startRow + bestRect.br().y));
            Imgproc.rectangle(outMat, subRect, new Scalar(255, 0, 0), 4);
            Imgproc.putText(blackMask, "Chosen", subRect.tl(), 0, 1, new Scalar(255, 255, 255));

            screenPosition = new Point(bestRect.x + bestRect.width / 2, bestRect.y + bestRect.height / 2);
            foundRect = bestRect;
            found = true;
        } else {
            found = false;
        }

        return outMat;
    }

    @Override
    public void useDefaults() {
        addScorer(ratioScorer);

        // Add diffrent scorers depending on the selected mode
        if (areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA) {
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA) {
            addScorer(perfectAreaScorer);
        }
    }
}
