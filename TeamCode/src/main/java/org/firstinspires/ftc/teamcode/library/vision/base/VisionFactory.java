package org.firstinspires.ftc.teamcode.library.vision.base;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.VuforiaKeyStore;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.jetbrains.annotations.NotNull;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class VisionFactory {

    public static VuforiaLocalizer createBasicVuforia(HardwareMap hardwareMap, String webcamName) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VuforiaKeyStore.getInstance().getVuforiaKey();
        parameters.cameraName = hardwareMap.get(WebcamName.class, webcamName);

        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
        CameraStreamSplitter.startCameraStream(vuforia,20);
        return vuforia;

    }

    public static TFObjectDetector attachFreightFrenzyTFOD(HardwareMap hardwareMap, VuforiaLocalizer vuforia, FreightFrenzyTFODModelSets modelSet) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        TFObjectDetector tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(modelSet.tfodModelAssetName, modelSet.labels);
        return tfod;
    }

    public static <Pipeline extends ResolutionPipeline> OpenCvContainer<Pipeline> createOpenCv(HardwareMap hardwareMap, String webcamName, @NotNull Pipeline pipeline) {
        OpenCvCamera camera;
        ImageResolution resolution = ImageResolution.R_960x720;
        OpenCvCameraRotation rotation = OpenCvCameraRotation.UPRIGHT;

        camera =
                OpenCvCameraFactory.getInstance().createWebcam(
                        hardwareMap.get(WebcamName.class, webcamName),
                        getCameraMonitorViewId(hardwareMap));
        return new OpenCvContainer<>(camera, pipeline, resolution, rotation);
    }

    private static int getCameraMonitorViewId(HardwareMap hardwareMap) {
        return hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    }

    public enum CameraType {
        WEBCAM_MINUS, WEBCAM_PLUS, PHONE_REAR, PHONE_FRONT
    }

    public enum FreightFrenzyTFODModelSets {
        ALL_ELEMENTS("FreightFrenzy_BCDM.tflite", "Ball", "Cube", "Duck", "Marker"),
        WAREHOUSE_ELEMENTS("FreightFrenzy_BC.tflite", "Ball", "Cube"),
        BARCODE_ELEMENTS("FreightFrenzy_DM.tflite", "Duck", "Marker");

        String tfodModelAssetName;
        String[] labels;

        private FreightFrenzyTFODModelSets(String tfodModelAssetName, String... labels) {
            this.tfodModelAssetName = tfodModelAssetName;
            this.labels = labels;
        }
    }
}
