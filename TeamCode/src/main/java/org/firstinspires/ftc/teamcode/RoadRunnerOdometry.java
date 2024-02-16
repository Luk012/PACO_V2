//import org.firstinspires.ftc.teamcode.FunnyLocalizer;
//import org.firstinspires.ftc.teamcode.Odometry as RRpose2d;
//
//public class RoadRunnerOdometry extends Odometry {
//    private FunnyLocalizer localizer;
//
//    public RoadRunnerOdometry(FunnyLocalizer localizer) {
//        super(convertToFTCLibPose(localizer.getPoseEstimate()));
//        this.localizer = localizer;
//    }
//
//    @Override
//    public void updatePose() {
//        this.robotPose = convertToFTCLibPose(localizer.getPoseEstimate());
//    }
//
//    @Override
//    public void updatePose(FtclibPose2d newPose) {
//        localizer.setPoseEstimate(convertToRRPose(newPose));
//        this.robotPose = newPose;
//    }
//
//    @Override
//    public FtclibPose2d getPose() {
//        updatePose(); // Ensure the latest pose is always returned
//        return robotPose;
//    }
//
//    private static FtclibPose2d convertToFTCLibPose(RRpose2d rrPose) {
//        return new FtclibPose2d(rrPose.getX(), rrPose.getY(), rrPose.getHeading());
//    }
//
//    private static RRpose2d convertToRRPose(FtclibPose2d ftclibPose) {
//        return new RRpose2d(ftclibPose.getX(), ftclibPose.getY(), ftclibPose.getHeading());
//    }
//}