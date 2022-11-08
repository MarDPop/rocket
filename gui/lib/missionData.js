
class Trajectory {

    constructor(){
        this.times = []
        this.position = []
    }
}

class MissionData {

    constructor() {
        this.trajectory = new Trajectory();
    }
}


module.exports = {Trajectory,MissionData}