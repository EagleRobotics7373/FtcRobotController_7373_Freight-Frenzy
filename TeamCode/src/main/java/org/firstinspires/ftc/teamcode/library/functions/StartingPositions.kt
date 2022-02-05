package org.firstinspires.ftc.teamcode.library.functions

enum class AllianceColor {
    RED, BLUE;

    companion object {
        @JvmField var persistingAllianceColor: AllianceColor = BLUE
    }
}

enum class AutonomousObjective {
    WAREHOUSE, CAROUSEL
}

enum class PostAllianceHubTask {
    WAREHOUSE, CAROUSEL, NOTHING, BACKPEDAL
}

enum class StartingPosition {
    NEAR_CAROUSEL, CENTER, NEAR_WAREHOUSE
}

enum class CameraPosition {
    LEFT, CENTER, RIGHT
}