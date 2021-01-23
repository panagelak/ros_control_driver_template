<div hidden>

  ```plantuml
  @startuml generic_driver_component_diagram
  skinparam componentStyle uml2
  skinparam defaultTextAlignment center

  left to right direction

  package "MoveIt Handlers" as moveit <<Folder>> {
    class  "MoveJointsHandlerServer" as movetojoints{
      -executeCallback()
    }
    class  "MoveToPoseHandlerServer" as movetopose{
      -executeCallback()
    }
  }


  package "Generic Hardware Interface" as gnHW <<Folder>> {
    abstract class  "HWBase" as base{
        +virtual read()
        +update()
        +virtual write()
    }
    interface  "GenericHWInterface" as interface{
        +read()
        +write()
    }
    class      "HWControlLoop" as cloop{
        +run()
        -update()
    }
  }

  package "Generic Driver" as gnDriver <<Folder>> {
    class "RobotDriver" as robot{
    +writeJointCommand()
    +getJointPosition()
    }
  }

  package "Servo Driver" as servoDriver <<Folder>> {
    abstract class  "ServoDriver" as servo{
        +subscribe()
        +write()
    }
  }



  ' Connections between classes

  ' Inside generic hardware
  base "inherits from" +--> interface
  interface -up-> cloop
  ' Moveit to hardware
  movetojoints *--> interface
  movetopose *--> interface
  ' Hardware Interface to Robot Driver
  interface --> robot
  ' Robot Driver TO TCP
  robot *--> servoDriver
  
  @enduml
  ```
  
</div>