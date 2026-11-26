"""Manual goal panel; uses the same external MOVE interface as a future WES."""
from concurrent.futures import ThreadPoolExecutor
from functools import partial
import asyncio
import math
import time
import uuid
from numbers import Real
from pi_car.transport.commands import send_command
from pi_car.transport.protocol import number, turn_degrees


class PanelModel:
    def __init__(self, grid, clearance_m=.2):
        self.grid, self.navigation = grid, grid.inflated(clearance_m)
        self.clearance = clearance_m
        self.info = None
        self.states = {}
        self.last_update = 0.

    def set_info(self, info):
        m = self.grid.metadata
        if (info['frame_id'] != m.frame_id or info['map_version'] != m.version
                or info['clearance_m'] != self.clearance):
            raise ValueError("Panel map/frame/clearance differs from PC server")
        if not info['robot_ids']:
            raise ValueError("Server has no robots")
        self.info = info

    def set_states(self, states):
        self.states = states
        self.last_update = time.monotonic()

    def _motion_pose(self, robot_id, states=None):
        if self.info is None or (states is None and time.monotonic()-self.last_update > 2.):
            raise ValueError("Wait for a current server connection")
        state = (self.states if states is None else states).get(robot_id)
        if not state or not state['connected']:
            raise ValueError("Selected robot is disconnected")
        pose = state.get('pose')
        if (not pose or not pose['valid'] or pose['frame_id'] != self.grid.metadata.frame_id
                or not 0 <= time.time()-pose['timestamp'] <= .3):
            reason = state.get('pose_error')
            message = "Selected robot has no fresh valid pose"
            raise ValueError(f"{message} ({reason})" if reason else message)
        return pose

    def move_request(self, robot_id, x, y, *, states=None):
        # Matplotlib events supply NumPy real scalars. Normalize at the GUI
        # boundary; keep the network protocol's strict JSON numeric validation.
        if any(isinstance(v,bool) or not isinstance(v,Real) for v in (x,y)):
            raise ValueError("Click coordinates must be real numbers, not booleans")
        x,y=float(x),float(y)
        number(x); number(y)
        self._motion_pose(robot_id, states)
        cell = self.navigation.world_to_cell(x,y)
        if not self.navigation.is_free(cell):
            raise ValueError("Choose free space outside obstacle/unknown/clearance regions")
        x,y = self.grid.cell_to_world(*cell)
        return dict(type='MOVE',robot_id=robot_id,command_id='manual.'+uuid.uuid4().hex,
                    frame_id=self.info['frame_id'],map_version=self.info['map_version'],goal=dict(x=x,y=y))

    def turn_request(self, robot_id, degrees, *, states=None):
        turn_degrees(degrees)
        pose = self._motion_pose(robot_id, states)
        if not self.navigation.is_free(self.navigation.world_to_cell(pose['x'], pose['y'])):
            raise ValueError("Vehicle is outside the clear turning area")
        return dict(type='TURN', robot_id=robot_id, command_id='manual.turn.'+uuid.uuid4().hex,
                    frame_id=self.info['frame_id'], map_version=self.info['map_version'],
                    degrees=degrees)

    def stop_request(self, robot_id):
        if self.info is None or robot_id not in self.info['robot_ids']:
            raise ValueError("Select a robot first")
        return dict(type='STOP',robot_id=robot_id,command_id='stop.'+uuid.uuid4().hex)


class FleetPanel:
    """GUI never blocks on TCP. One query + one command in flight; no goal queue."""
    def __init__(self, grid, *, host='127.0.0.1', port=8766, clearance_m=.2):
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap,BoundaryNorm
        from matplotlib.widgets import Button, TextBox
        import numpy as np
        self.plt = plt
        self.model = PanelModel(grid,clearance_m)
        self.host,self.port = host,port
        self.pool = ThreadPoolExecutor(max_workers=2,thread_name_prefix='panel-tcp')
        self.pending = None
        self.command_pending = None
        self.closed = False
        self.selected = None
        self.artists = {}
        self.radio = None
        self.fig,self.ax = plt.subplots(figsize=(11,8))
        self.fig.subplots_adjust(left=.12,right=.74,bottom=.23)
        m=grid.metadata
        extent=[m.origin_xy[0],m.origin_xy[0]+m.width*m.resolution_m,
                m.origin_xy[1],m.origin_xy[1]+m.height*m.resolution_m]
        self.ax.imshow(grid.cells,origin='lower',extent=extent,
                        cmap=ListedColormap(['#959ba3','#fafafa','#242a32']),
                        norm=BoundaryNorm([-1.5,-.5,.5,1.5],3))
        blocked=np.ma.masked_where((grid.cells!=0)|(self.model.navigation.cells==0),np.ones(grid.cells.shape))
        self.ax.imshow(blocked,origin='lower',extent=extent,cmap=ListedColormap(['#edb56f']),
                        alpha=.7,vmin=0,vmax=1)
        self.ax.set(xlabel='World X (m)',ylabel='World Y (m)',
                    title='Select robot, then LEFT CLICK destination (sends MOVE immediately)')
        self.text=self.fig.text(.76,.68,'Connecting...',va='top',fontsize=9)
        self.notice=self.fig.text(.06,.1,'Gray unknown | Black obstacle | Orange vehicle clearance',fontsize=10,wrap=True)
        self.fig.text(.76,.535,'Degrees',fontsize=10)
        self.turn_angle=TextBox(self.fig.add_axes([.84,.515,.12,.045]),'',initial='25')
        self.turn_left=Button(self.fig.add_axes([.76,.445,.095,.05]),'Turn left')
        self.turn_right=Button(self.fig.add_axes([.865,.445,.095,.05]),'Turn right')
        self.turn_left.on_clicked(lambda _:self._turn(1))
        self.turn_right.on_clicked(lambda _:self._turn(-1))
        self.stop_button=Button(self.fig.add_axes([.76,.3,.2,.06]),'STOP selected',color='#f4aaaa')
        self.stop_button.on_clicked(self._stop)
        self.fig.canvas.mpl_connect('button_press_event',self._click)
        self.fig.canvas.mpl_connect('close_event',lambda _:self.close())
        self.fig.canvas.mpl_connect('key_press_event',lambda e:self._stop(None) if e.key=='escape' else None)
        self.timer=self.fig.canvas.new_timer(interval=100)
        self.timer.add_callback(self.refresh)

    def _submit(self, request, *, motion_request=None):
        field='pending' if request['type'] in {'INFO','STATUS'} else 'command_pending'
        if getattr(self,field) is not None:
            self._notice('Network request in progress; click again after it finishes')
            return False
        async def send():
            if motion_request is None:
                return await send_command(self.host, self.port, request)
            status = await send_command(self.host, self.port, dict(type='STATUS'))
            if not status['ok']:
                return status
            if self.closed:
                raise ValueError("Panel closed before sending the command")
            command = motion_request(states=status['result'])
            return await send_command(self.host, self.port, command)

        future=self.pool.submit(lambda:asyncio.run(send()))
        setattr(self,field,(request,future))
        return True

    def _notice(self,message):
        self.notice.set_text(message)
        self.fig.canvas.draw_idle()

    def _select(self,robot_id):
        self.selected=robot_id

    def _click(self,event):
        if event.inaxes!=self.ax or event.button!=1 or event.xdata is None or event.ydata is None:
            return
        toolbar=getattr(self.fig.canvas,'toolbar',None)
        if toolbar and toolbar.mode:
            return
        robot_id, x, y = self.selected, event.xdata, event.ydata
        request = partial(self.model.move_request, robot_id, x, y)
        if self._submit(dict(type='MOVE'), motion_request=request):
            self._notice(f"Checking current position for vehicle {robot_id}…")

    def _stop(self,_):
        try:
            self._submit(self.model.stop_request(self.selected))
        except ValueError as error:self._notice(str(error))

    def _turn(self, direction):
        try:
            degrees = float(self.turn_angle.text)
            if degrees <= 0:
                raise ValueError("Enter a positive turn angle in degrees")
            robot_id = self.selected
            request = partial(self.model.turn_request, robot_id, direction * degrees)
            if self._submit(dict(type='TURN'), motion_request=request):
                self._notice(f"Checking current position for vehicle {robot_id}…")
        except ValueError as error:
            self._notice(str(error))

    def _configure(self):
        from matplotlib.widgets import RadioButtons
        robots=self.model.info['robot_ids']
        self.radio=RadioButtons(self.fig.add_axes([.76,.73,.2,.16]),robots)
        self.radio.on_clicked(self._select)
        self.selected=robots[0]
        for i,r in enumerate(robots):
            color=self.plt.get_cmap('tab10')(i%10)
            point,=self.ax.plot([],[],'o',color=color,label=r)
            heading,=self.ax.plot([],[],'-',color=color)
            goal,=self.ax.plot([],[],'*',color=color,markersize=14)
            self.artists[r]=(point,heading,goal)
        self.ax.legend()

    def _draw_states(self):
        lines=[]
        for r,(point,heading,goal_artist) in self.artists.items():
            state=self.model.states.get(r,{})
            pose=state.get('pose')
            if (state.get('connected') and pose and 0 <= time.time()-pose['timestamp'] <= .3
                    and time.monotonic()-self.model.last_update <= 2.):
                x,y,t=pose['x'],pose['y'],pose['theta']
                point.set_data([x],[y]);heading.set_data([x,x+.15*math.cos(t)],[y,y+.15*math.sin(t)])
            else:
                point.set_data([],[]);heading.set_data([],[])
            goal=state.get('goal')
            goal_artist.set_data([goal['x']],[goal['y']]) if goal else goal_artist.set_data([],[])
            result=state.get('last_result') or {}
            command=state.get('command_id') or '-'
            if len(command)>24:command=command[:21]+'...'
            reason=state.get('pose_error') or result.get('reason','')
            if len(reason)>28:reason=reason[:25]+'...'
            line = f"{r}: {state.get('status','OFFLINE')}\n{command}\n{reason}"
            if 'degrees' in result:
                line += f"\nTurn {result['degrees']:+g}°"
                if 'turned_degrees' in result:
                    line += f" → {result['turned_degrees']:+.1f}°"
            lines.append(line)
        self.text.set_text('\n\n'.join(lines))
        self.fig.canvas.draw_idle()

    def refresh(self):
        if self.closed:return
        for field in ('pending','command_pending'):
            entry=getattr(self,field)
            if not entry:continue
            request,future=entry
            if future.done():
                setattr(self,field,None)
                try:
                    response=future.result()
                    if not response['ok']:raise ValueError(response['error'])
                    if request['type']=='INFO':
                        self.model.set_info(response['result']);self._configure()
                    elif request['type']=='STATUS':self.model.set_states(response['result'])
                    else:self._notice(f"{request['type']}: {response['result']['type']} (watch robot status)")
                except (OSError,ValueError,KeyError,RuntimeError) as error:
                    if field == 'pending':
                        self.model.states={}
                    self._notice(f'Connection/command error: {error}')
        if self.pending is None:
            self._submit(dict(type='STATUS' if self.model.info else 'INFO'))
        self._draw_states()

    def show(self):
        if self.plt.get_backend().lower() in {'agg','pdf','svg','ps','cairo','template'}:
            self.close()
            raise RuntimeError('Panel needs an interactive desktop backend (e.g. TkAgg / python3-tk)')
        self.timer.start()
        try:self.plt.show()
        finally:self.close()

    def close(self):
        if not self.closed:
            self.closed=True
            self.timer.stop()
            self.pool.shutdown(wait=False,cancel_futures=True)
