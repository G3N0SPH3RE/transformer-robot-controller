import React, { Component } from 'react';
import styled from "styled-components";
import server_config from "./config/machine_specific_server_config.json";
import Switch from 'react-switch';



import './App.css'


const ButtonGroup = styled.div`
  display: flex;
  padding: 20px 60px;
  margin: 10px;
  justify-content: center;
  align-items: center;
`

const Button = styled.button`
  background-color: black;
  color: white;
  font-size: 15px;
  padding: 10px 30px;
  border-radius: 5px;
  margin: 10px 10px;
  cursor: pointer;
`;

const AxisLabelHeader = styled.div`
  display: flex;
  justify-content:center; // centers in the flex direction and the default flex-direction is row
  align-items:center; // centers perpendicular to the flex direction
  //height: 100vh; // 100% view height
  width: 100vw; // 100% view width
  position: absolute; // so it goes behind the current content
  color:black;
  font-weight:bold;
  font-family:    Arial, Helvetica, sans-serif;
  font-size:      40px;
`;

class App extends Component {
  constructor () {
    super();
    this.handleClick = this.handleClick.bind(this);
    this.connect = this.connect.bind(this);

    this.state = {
      x_position: 0,
      y_position: 0,
      z_position: 0,
      c_position: 0
    }
  }

  connect() {
    const client = new WebSocket('ws://' + server_config.SERVER_ADDRESS + ':' + server_config.SERVER_PORT + '/ws');
    client.onmessage = this.onMessage;

    client.onerror = this.onError;
    client.onclose = this.onClose;

    this.setState({
      client: client,
    })
  }

  componentDidMount() {
    this.connect();
  }

  componentWillUnmount() {
    const {client, interval} = this.state;
    client.close()
    //clearInterval(interval)
  }

  onMessage = (ev) => {
    console.log(ev.data);
    const recv = JSON.parse(ev.data);
    for (var key in recv) {
      this.setState({
        [key]: recv[key]
      })
    };
  }

  onError = (error) => {
    console.error("websocket error: " + error.message);
  }

  onClose = (e) => {
    console.log('Socket is closed. Reconnect will be attempted in 1 second.', e.reason);
    setTimeout(this.connect(), 1000);
  }

  handleMachineStateCommand(command) {
    console.log("Machine command is " + command);
    const requestOptions = {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ "state": command })
    };

    function handleError (response) {
      console.log('response ' + response.ok);
      if (!response.ok) {
        throw Error(response.statusText)
      }
      return response;
    }

    fetch('http://' + server_config.SERVER_ADDRESS + ':' + server_config.SERVER_PORT + '/machine/state', requestOptions)
          .then(handleError)
          .then(function(response) {
            console.log(response.success_string)
            console.log("ok");
          })
          .catch(function(error) {
            console.log(error)
          })
  }

  handleMachineDriveCommand(command) {
    console.log("Machine command is " + command);
    const requestOptions = {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ "state": command })
    };

    function handleError (response) {
      console.log('response ' + response.ok);
      if (!response.ok) {
        throw Error(response.statusText)
      }
      return response;
    }

    fetch('http://' + server_config.SERVER_ADDRESS + ':' + server_config.SERVER_PORT + '/machine/' + command, requestOptions)
          .then(handleError)
          .then(function(response) {
            console.log(response.success_string)
            console.log("ok");
          })
          .catch(function(error) {
            console.log(error)
          })
  }


  handleClick (jog_axis, jog_amount) {
    console.log('Success!');
    console.log('Jog axis is ' + jog_axis + ' and jog amount is ' + jog_amount.toString());
    
    const requestOptions = {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ "target_position": 0 })
    };
    

    function handleError (response) {
      console.log('response ' + response.ok);
      if (!response.ok) {
        throw Error(response.statusText)
      }
      return response;
    }

    requestOptions.body = JSON.stringify({ "target_position": jog_amount })
    fetch('http://' + server_config.SERVER_ADDRESS + ':' + server_config.SERVER_PORT + '/axes/' + jog_axis.toLowerCase() + '/motion/relative', requestOptions)
          .then(handleError)
          .then(function(response) {
            console.log(response.success_string)
            console.log("ok");
          })
          .catch(function(error) {
            console.log(error)
          })
  }


  //Gripper//
  handleMachineGripperCommand(command) {
    console.log("Machine command is " + command);
    const requestOptions = {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ "gripper_state": command })
    };

    function handleError (response) {
      console.log('response ' + response.ok);
      if (!response.ok) {
        throw Error(response.statusText)
      }
      return response;
    }

    fetch('http://' + server_config.SERVER_ADDRESS + ':' + server_config.SERVER_PORT + '/machine/gripper', requestOptions)
          .then(handleError)
          .then(function(response) {
            console.log(response.success_string)
            console.log("ok");
          })
          .catch(function(error) {
            console.log(error)
          })
  }
  //Tool Clamping State//
    handleMachineToolCommand(command) {
    console.log("Machine command is " + command);
    const requestOptions = {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ "point_state": command })
    };

    function handleError (response) {
      console.log('response ' + response.ok);
      if (!response.ok) {
        throw Error(response.statusText)
      }
      return response;
    }

    fetch('http://' + server_config.SERVER_ADDRESS + ':' + server_config.SERVER_PORT + '/axes/x/io/output/0', requestOptions)
          .then(handleError)
          .then(function(response) {
            console.log(response.success_string)
            console.log("ok");
          })
          .catch(function(error) {
            console.log(error)
          })
  }
  //Air Connect//
    handleMachineAirConnectCommand(command) {
    console.log("Machine command is " + command);
    const requestOptions = {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ "point_state": command })
    };

    function handleError (response) {
      console.log('response ' + response.ok);
      if (!response.ok) {
        throw Error(response.statusText)
      }
      return response;
    }

    fetch('http://' + server_config.SERVER_ADDRESS + ':' + server_config.SERVER_PORT + '/axes/x/io/output/1', requestOptions)
          .then(handleError)
          .then(function(response) {
            console.log(response.success_string)
            console.log("ok");
          })
          .catch(function(error) {
            console.log(error)
          })
  }
  ////Test Start////

//// Toggle Switch ////


  ////Test End////

//// --UI-- ////
  render () {
    return (
      <div className='button__container'>
        <AxisLabelHeader>Positions</AxisLabelHeader>
        <br></br>
        <table class="centerTable">
          <tr>
            <td class="centerTableCell">X: {this.state.x_position.toFixed(4)}</td>
            <td class="centerTableCell">Y: {this.state.y_position.toFixed(4)}</td>
            <td class="centerTableCell">Z: {this.state.y_position.toFixed(4)}</td>
            <td class="centerTableCell">C: {this.state.y_position.toFixed(4)}</td>
          </tr>
        </table>

        <AxisLabelHeader>Machine</AxisLabelHeader>
        <br></br>
        <ButtonGroup>
          <Button onClick={() => this.handleMachineStateCommand("abort")}>
            ABORT
          </Button>
          <Button onClick={() => this.handleMachineStateCommand("stop")}>
            STOP
          </Button>
          <Button onClick={() => this.handleMachineStateCommand("manual")}>
            MANUAL
          </Button>
        </ButtonGroup>

        <AxisLabelHeader>Drives</AxisLabelHeader>
        <br></br>
        <ButtonGroup>
          <Button onClick={() => this.handleMachineDriveCommand("reset")}>
            RESET ALL
          </Button>
          <Button onClick={() => this.handleMachineDriveCommand("enable")}>
            ENABLE ALL
          </Button>
        </ButtonGroup>

      <br></br>
      <label>
        Switch with default style
        <Switch onChange={this.handleChange} checked={this.state.checked} />
      </label>


        <AxisLabelHeader>Gripper</AxisLabelHeader>
        <br></br>
        <ButtonGroup>
          <Button onClick={() => this.handleMachineGripperCommand("open")}>
            OPEN
          </Button>
          <Button onClick={() => this.handleMachineGripperCommand("close")}>
            CLOSE
          </Button>
        </ButtonGroup>

        <br></br>
        <ButtonGroup>
          <Button onClick={() => this.handleMachineToolCommand("0")}>
            TOOL CLAMP
          </Button>
          <Button onClick={() => this.handleMachineToolCommand("1")}>
            TOOL UNCLAMP
          </Button>
        </ButtonGroup>

        <AxisLabelHeader>Air Connect</AxisLabelHeader>
        <br></br>
        <ButtonGroup>
          <Button onClick={() => this.handleMachineAirConnectCommand("0")}>
            DISENGAGE
          </Button>
          <Button onClick={() => this.handleMachineAirConnectCommand("1")}>
            ENGAGE
          </Button>
        </ButtonGroup>

        <AxisLabelHeader>X Axis</AxisLabelHeader>
        <br></br>
        <ButtonGroup>
          <Button onClick={() => this.handleClick("x", 10.0)}>
            Jog +10mm
          </Button>
          <Button onClick={() => this.handleClick("x", -10.0)}>
            Jog -10mm
          </Button>
        </ButtonGroup>

        <AxisLabelHeader>Z Axis</AxisLabelHeader>
        <br></br>
        <ButtonGroup>
          <Button onClick={() => this.handleClick("z", 10.0)}>
            Jog +10mm
          </Button>
          <Button onClick={() => this.handleClick("z", -10.0)}>
            Jog -10mm
          </Button>
        </ButtonGroup>

        <AxisLabelHeader>C Axis</AxisLabelHeader>
        <br></br>
        <ButtonGroup>
          <Button name="Cplus" onClick={() => this.handleClick("c", 1.0)}>
            Jog +1deg
          </Button>
          <Button onClick={() => this.handleClick("c", -1.0)}>
            Jog -1deg
          </Button>
        </ButtonGroup>

      </div>
    )
  }
}
export default App