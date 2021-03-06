/****************************************************************************
 *
 *   (c) 2009-2016 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QtQuick          2.3
import QtQuick.Controls 1.2
import QtCharts         2.2
import QtQuick.Layouts  1.2

import QGroundControl               1.0
import QGroundControl.Controls      1.0
import QGroundControl.FactSystem    1.0
import QGroundControl.FactControls  1.0
import QGroundControl.ScreenTools   1.0

SetupPage {
    id:             tuningPage
    pageComponent:  pageComponent

    Component {
        id: pageComponent

        Column {
            width: availableWidth

            Component.onCompleted: {
                showAdvanced = !ScreenTools.isMobile
            }

            FactPanelController {
                id:         controller
                factPanel:  tuningPage.viewPanel
            }

            // Standard tuning page
            FactSliderPanel {
                width:          availableWidth
                qgcViewPanel:   tuningPage.viewPanel
                visible:        !advanced

                sliderModel: ListModel {
                    ListElement {
                        title:          qsTr("Hover Throttle")
                        description:    qsTr("Adjust throttle so hover is at mid-throttle. Slide to the left if hover is lower than throttle center. Slide to the right if hover is higher than throttle center.")
                        param:          "MPC_THR_HOVER"
                        min:            20
                        max:            80
                        step:           1
                    }

                    ListElement {
                        title:          qsTr("Manual minimum throttle")
                        description:    qsTr("Slide to the left to start the motors with less idle power. Slide to the right if descending in manual flight becomes unstable.")
                        param:          "MPC_MANTHR_MIN"
                        min:            0
                        max:            15
                        step:           1
                    }
                    /*
  These seem to have disappeared from PX4 firmware!
                    ListElement {
                        title:          qsTr("Roll sensitivity")
                        description:    qsTr("Slide to the left to make roll control faster and more accurate. Slide to the right if roll oscillates or is too twitchy.")
                        param:          "MC_ROLL_TC"
                        min:            0.15
                        max:            0.25
                        step:           0.01
                    }

                    ListElement {
                        title:          qsTr("Pitch sensitivity")
                        description:    qsTr("Slide to the left to make pitch control faster and more accurate. Slide to the right if pitch oscillates or is too twitchy.")
                        param:          "MC_PITCH_TC"
                        min:            0.15
                        max:            0.25
                        step:           0.01
                    }
*/
                }
            }

            Loader {
                anchors.left:       parent.left
                anchors.right:      parent.right
                sourceComponent:    advanced ? advancePageComponent : undefined
            }

            Component {
                id: advancePageComponent

                PIDTuning {
                    anchors.left:   parent.left
                    anchors.right:  parent.right
                    tuneList:            [ qsTr("Roll"), qsTr("Pitch"), qsTr("Yaw") ]
                    params:              [
                        [ controller.getParameterFact(-1, "MC_ROLL_P"),
                         controller.getParameterFact(-1, "MC_ROLLRATE_P"),
                         controller.getParameterFact(-1, "MC_ROLLRATE_I"),
                         controller.getParameterFact(-1, "MC_ROLLRATE_D"),
                         controller.getParameterFact(-1, "MC_ROLLRATE_FF") ],
                        [ controller.getParameterFact(-1, "MC_PITCH_P"),
                         controller.getParameterFact(-1, "MC_PITCHRATE_P"),
                         controller.getParameterFact(-1, "MC_PITCHRATE_I"),
                         controller.getParameterFact(-1, "MC_PITCHRATE_D"),
                         controller.getParameterFact(-1, "MC_PITCHRATE_FF") ],
                        [ controller.getParameterFact(-1, "MC_YAW_P"),
                         controller.getParameterFact(-1, "MC_YAWRATE_P"),
                         controller.getParameterFact(-1, "MC_YAWRATE_I"),
                         controller.getParameterFact(-1, "MC_YAWRATE_D"),
                         controller.getParameterFact(-1, "MC_YAWRATE_FF") ] ]
                }
            } // Component - Advanced Page
        } // Column
    } // Component - pageComponent
} // SetupPage
