use std::{
    collections::VecDeque,
    io::{self, Stdout},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::Duration,
};

use crossterm::{
    cursor,
    event::{
        self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEvent, KeyEventKind,
        KeyModifiers, MouseButton, MouseEvent, MouseEventKind,
    },
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use libmonado::{self as mnd, DeviceLogic};
use libmotoc::{CalibratorData, ResultExt};
use log::Level;
use nalgebra::{Quaternion, Rotation3, UnitQuaternion, Vector3};
use openxr::SpaceVelocityFlags;
use ratatui::{
    backend::CrosstermBackend,
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Clear, List, ListItem, ListState, Paragraph},
    Frame, Terminal,
};

use crate::{OffsetType, TransformD};

use super::{CalibratorStatus, OffsetMethod, RecenterMethod, SampledMethod, StepResult};

pub type Result<T> = std::result::Result<T, libmotoc::Error>;

const TICKER_SIZE: usize = 10;
const MAX_LOG_LINES: usize = 200;
pub const SPINNER_TICK_CHARS: &str = "⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏";
const COMMANDS: [&str; 5] = ["Continue", "Calibrate", "Adjust", "Recenter", "Reset"];
const OFFSET_DELTAS: [f64; 8] = [-10.0, -1.0, -0.1, -0.01, 0.01, 0.1, 1.0, 10.0];

type TuiTerminal = Terminal<CrosstermBackend<Stdout>>;

#[derive(Clone, Debug)]
struct LogLine {
    level: Level,
    message: String,
}

#[derive(Default)]
struct TuiLogBufferInner {
    active: AtomicBool,
    lines: Mutex<VecDeque<LogLine>>,
}

#[derive(Clone, Default)]
pub struct TuiLogBuffer {
    inner: Arc<TuiLogBufferInner>,
}

impl TuiLogBuffer {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&self, level: Level, message: &str) {
        let mut lines = self
            .inner
            .lines
            .lock()
            .unwrap_or_else(|error| error.into_inner());

        for message in message.split('\n') {
            lines.push_back(LogLine {
                level,
                message: message.trim_end_matches('\r').to_owned(),
            });
        }

        while lines.len() > MAX_LOG_LINES {
            lines.pop_front();
        }
    }

    pub fn is_active(&self) -> bool {
        self.inner.active.load(Ordering::Acquire)
    }

    fn set_active(&self, active: bool) {
        self.inner.active.store(active, Ordering::Release);
    }

    fn recent(&self, count: usize) -> Vec<LogLine> {
        let lines = self
            .inner
            .lines
            .lock()
            .unwrap_or_else(|error| error.into_inner());
        let skip = lines.len().saturating_sub(count);
        lines.iter().skip(skip).cloned().collect()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SpaceKind {
    Stage,
    Local,
}

impl SpaceKind {
    fn name(self) -> &'static str {
        match self {
            Self::Stage => "STAGE",
            Self::Local => "LOCAL",
        }
    }

    fn argument(self) -> &'static str {
        match self {
            Self::Stage => "stage",
            Self::Local => "local",
        }
    }

    fn reference_type(self) -> mnd::ReferenceSpaceType {
        match self {
            Self::Stage => mnd::ReferenceSpaceType::Stage,
            Self::Local => mnd::ReferenceSpaceType::Local,
        }
    }
}

#[derive(Clone, Debug)]
enum AdjustTarget {
    Space(SpaceKind),
    TrackingOrigin { id: u32, name: String },
}

impl AdjustTarget {
    fn label(&self) -> String {
        match self {
            Self::Space(space) => format!("{} space", space.name()),
            Self::TrackingOrigin { id, name } if name.is_empty() => {
                format!("[{id}] Unknown tracking origin")
            }
            Self::TrackingOrigin { id, name } => format!("[{id}] {name}"),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Axis {
    X,
    Y,
    Z,
    Pitch,
    Yaw,
    Roll,
}

impl Axis {
    const ALL: [Self; 6] = [
        Self::X,
        Self::Y,
        Self::Z,
        Self::Pitch,
        Self::Yaw,
        Self::Roll,
    ];

    fn name(self) -> &'static str {
        match self {
            Self::X => "X",
            Self::Y => "Y",
            Self::Z => "Z",
            Self::Pitch => "Pitch",
            Self::Yaw => "Yaw",
            Self::Roll => "Roll",
        }
    }

    fn unit(self) -> &'static str {
        match self {
            Self::X | Self::Y | Self::Z => "m",
            Self::Pitch | Self::Yaw | Self::Roll => "deg",
        }
    }

    fn index(self) -> usize {
        Self::ALL.iter().position(|axis| *axis == self).unwrap_or(0)
    }

    fn from_index(index: usize) -> Self {
        Self::ALL[index.min(Self::ALL.len() - 1)]
    }
}

#[derive(Clone, Debug)]
struct CalibrateForm {
    source: usize,
    target: usize,
    samples: String,
    continuous: bool,
    selected: usize,
    editing_samples: bool,
}

#[derive(Clone, Debug)]
enum Screen {
    Dashboard,
    Calibrate(CalibrateForm),
    AdjustSelect {
        selected: usize,
    },
    AdjustEdit {
        target: AdjustTarget,
        axis: Axis,
        selected_delta: usize,
    },
    Recenter {
        selected: usize,
    },
    Reset {
        selected: usize,
    },
}

#[derive(Clone, Debug)]
enum MouseAction {
    Command(usize),
    CalibrateCycleDevice { source: bool, delta: isize },
    CalibrateSamples(i32),
    CalibrateToggle,
    CalibrateStart,
    AdjustTarget(usize),
    AdjustDelta { axis: Axis, delta: f64 },
    Recenter(SpaceKind),
    Reset(usize),
}

#[derive(Clone, Debug)]
struct Hitbox {
    rect: Rect,
    action: MouseAction,
}

impl Hitbox {
    fn contains(&self, column: u16, row: u16) -> bool {
        column >= self.rect.x
            && column < self.rect.x.saturating_add(self.rect.width)
            && row >= self.rect.y
            && row < self.rect.y.saturating_add(self.rect.height)
    }
}

pub struct Tui {
    terminal: Option<TuiTerminal>,
    terminal_active: bool,
    logs: TuiLogBuffer,
    spinner_frame: usize,
    screen: Screen,
    selected_command: usize,
    overview_scroll: u16,
    hitboxes: Vec<Hitbox>,
    overview_area: Rect,
    status: String,
}

impl Tui {
    pub fn new(logs: TuiLogBuffer) -> Self {
        Self {
            terminal: None,
            terminal_active: false,
            logs,
            spinner_frame: 0,
            screen: Screen::Dashboard,
            selected_command: 0,
            overview_scroll: 0,
            hitboxes: Vec::new(),
            overview_area: Rect::default(),
            status: String::new(),
        }
    }

    fn start_terminal(&mut self) -> Result<()> {
        self.logs.set_active(true);
        if let Err(error) = enable_raw_mode() {
            self.logs.set_active(false);
            return Err(error).context("Unable to enable terminal raw mode");
        }

        let mut stdout = io::stdout();
        if let Err(error) = execute!(
            stdout,
            EnterAlternateScreen,
            EnableMouseCapture,
            cursor::Hide
        ) {
            let _ = disable_raw_mode();
            self.logs.set_active(false);
            return Err(error).context("Unable to enter the alternate screen");
        }

        match Terminal::new(CrosstermBackend::new(stdout)) {
            Ok(terminal) => {
                self.terminal = Some(terminal);
                self.terminal_active = true;
                if let Err(error) = self
                    .terminal
                    .as_mut()
                    .expect("terminal was just initialized")
                    .clear()
                {
                    let _ = self.stop_terminal();
                    return Err(error).context("Unable to clear the terminal");
                }
                Ok(())
            }
            Err(error) => {
                let mut stdout = io::stdout();
                let _ = execute!(
                    stdout,
                    cursor::Show,
                    DisableMouseCapture,
                    LeaveAlternateScreen
                );
                let _ = disable_raw_mode();
                self.logs.set_active(false);
                Err(error).context("Unable to initialize the terminal")
            }
        }
    }

    fn stop_terminal(&mut self) -> Result<()> {
        if !self.terminal_active {
            self.logs.set_active(false);
            return Ok(());
        }

        self.terminal_active = false;
        let mut first_error = None;

        if let Some(terminal) = self.terminal.as_mut() {
            if let Err(error) = execute!(
                terminal.backend_mut(),
                cursor::Show,
                DisableMouseCapture,
                LeaveAlternateScreen
            ) {
                first_error = Some(error);
            }
            let _ = terminal.show_cursor();
        }
        self.terminal = None;

        if let Err(error) = disable_raw_mode() {
            first_error.get_or_insert(error);
        }
        self.logs.set_active(false);

        match first_error {
            Some(error) => Err(error).context("Unable to restore the terminal"),
            None => Ok(()),
        }
    }

    fn draw(
        &mut self,
        data: &CalibratorData<'_>,
        calibrator_status: Option<&CalibratorStatus>,
    ) -> Result<()> {
        self.hitboxes.clear();

        let spinner_char = if matches!(calibrator_status, Some(CalibratorStatus::Spinner { .. })) {
            let spinner_char = SPINNER_TICK_CHARS
                .chars()
                .nth(self.spinner_frame)
                .unwrap_or('⠋');
            self.spinner_frame = (self.spinner_frame + 1) % SPINNER_TICK_CHARS.chars().count();
            spinner_char
        } else {
            self.spinner_frame = 0;
            '⠋'
        };

        let screen = &self.screen;
        let selected_command = self.selected_command;
        let overview_scroll = self.overview_scroll;
        let status = &self.status;
        let logs = &self.logs;
        let hitboxes = &mut self.hitboxes;
        let overview_area = &mut self.overview_area;

        if let Some(terminal) = self.terminal.as_mut() {
            terminal
                .draw(|frame| {
                    draw_ui(
                        frame,
                        data,
                        screen,
                        selected_command,
                        overview_scroll,
                        status,
                        calibrator_status,
                        spinner_char,
                        logs,
                        hitboxes,
                        overview_area,
                    )
                })
                .context("Unable to draw the TUI")?;
        }

        Ok(())
    }

    fn handle_key(&mut self, key: KeyEvent, data: &mut CalibratorData<'_>) -> StepResult {
        if key.kind != KeyEventKind::Press {
            return StepResult::Continue;
        }

        if key.modifiers.contains(KeyModifiers::CONTROL) && matches!(key.code, KeyCode::Char('c')) {
            return StepResult::End;
        }

        match self.screen.clone() {
            Screen::Dashboard => match key.code {
                KeyCode::Char('q') | KeyCode::Esc => StepResult::End,
                KeyCode::Up | KeyCode::Char('k') => {
                    self.selected_command = self.selected_command.saturating_sub(1);
                    StepResult::Continue
                }
                KeyCode::Down | KeyCode::Char('j') => {
                    self.selected_command = (self.selected_command + 1).min(COMMANDS.len() - 1);
                    StepResult::Continue
                }
                KeyCode::Enter | KeyCode::Char(' ') => {
                    self.activate_command(self.selected_command, data)
                }
                KeyCode::PageUp => {
                    self.overview_scroll = self.overview_scroll.saturating_sub(5);
                    StepResult::Continue
                }
                KeyCode::PageDown => {
                    self.overview_scroll = self.overview_scroll.saturating_add(5);
                    StepResult::Continue
                }
                _ => StepResult::Continue,
            },
            Screen::Calibrate(mut form) => match key.code {
                KeyCode::Esc => {
                    self.screen = Screen::Dashboard;
                    StepResult::Continue
                }
                KeyCode::Up | KeyCode::Char('k') => {
                    form.selected = form.selected.saturating_sub(1);
                    form.editing_samples = false;
                    self.screen = Screen::Calibrate(form);
                    StepResult::Continue
                }
                KeyCode::Down | KeyCode::Char('j') | KeyCode::Tab => {
                    form.selected = (form.selected + 1).min(5);
                    form.editing_samples = false;
                    self.screen = Screen::Calibrate(form);
                    StepResult::Continue
                }
                KeyCode::BackTab => {
                    form.selected = form.selected.saturating_sub(1);
                    form.editing_samples = false;
                    self.screen = Screen::Calibrate(form);
                    StepResult::Continue
                }
                KeyCode::Left => {
                    self.adjust_calibrate_field(&mut form, data, -1);
                    self.screen = Screen::Calibrate(form);
                    StepResult::Continue
                }
                KeyCode::Right => {
                    self.adjust_calibrate_field(&mut form, data, 1);
                    self.screen = Screen::Calibrate(form);
                    StepResult::Continue
                }
                KeyCode::Backspace if form.selected == 2 => {
                    form.samples.pop();
                    form.editing_samples = true;
                    self.screen = Screen::Calibrate(form);
                    StepResult::Continue
                }
                KeyCode::Char(ch) if form.selected == 2 && ch.is_ascii_digit() => {
                    if !form.editing_samples {
                        form.samples.clear();
                        form.editing_samples = true;
                    }
                    if form.samples.len() < 9 {
                        form.samples.push(ch);
                    }
                    self.screen = Screen::Calibrate(form);
                    StepResult::Continue
                }
                KeyCode::Enter | KeyCode::Char(' ') => match form.selected {
                    0 => {
                        cycle_index(&mut form.source, data.devices.len(), 1);
                        self.screen = Screen::Calibrate(form);
                        StepResult::Continue
                    }
                    1 => {
                        cycle_index(&mut form.target, data.devices.len(), 1);
                        self.screen = Screen::Calibrate(form);
                        StepResult::Continue
                    }
                    2 => {
                        form.editing_samples = true;
                        self.screen = Screen::Calibrate(form);
                        StepResult::Continue
                    }
                    3 => {
                        form.continuous = !form.continuous;
                        self.screen = Screen::Calibrate(form);
                        StepResult::Continue
                    }
                    4 => self.start_calibration(&form, data),
                    _ => {
                        self.screen = Screen::Dashboard;
                        StepResult::Continue
                    }
                },
                _ => StepResult::Continue,
            },
            Screen::AdjustSelect { mut selected } => {
                let targets = offset_targets(data);
                match key.code {
                    KeyCode::Esc => {
                        self.screen = Screen::Dashboard;
                    }
                    KeyCode::Up | KeyCode::Char('k') => {
                        selected = selected.saturating_sub(1);
                        self.screen = Screen::AdjustSelect { selected };
                    }
                    KeyCode::Down | KeyCode::Char('j') => {
                        selected = (selected + 1).min(targets.len().saturating_sub(1));
                        self.screen = Screen::AdjustSelect { selected };
                    }
                    KeyCode::Enter | KeyCode::Char(' ') => {
                        if let Some(target) = targets.get(selected).cloned() {
                            self.screen = Screen::AdjustEdit {
                                target,
                                axis: Axis::X,
                                selected_delta: 4,
                            };
                        }
                    }
                    _ => {}
                }
                StepResult::Continue
            }
            Screen::AdjustEdit {
                target,
                mut axis,
                mut selected_delta,
            } => {
                match key.code {
                    KeyCode::Esc => {
                        self.screen = Screen::AdjustSelect { selected: 0 };
                        return StepResult::Continue;
                    }
                    KeyCode::Up | KeyCode::Char('k') => {
                        axis = Axis::from_index(axis.index().saturating_sub(1));
                    }
                    KeyCode::Down | KeyCode::Char('j') => {
                        axis = Axis::from_index((axis.index() + 1).min(Axis::ALL.len() - 1));
                    }
                    KeyCode::Left | KeyCode::Char('h') => {
                        selected_delta = selected_delta.saturating_sub(1);
                    }
                    KeyCode::Right | KeyCode::Char('l') => {
                        selected_delta = (selected_delta + 1).min(OFFSET_DELTAS.len() - 1);
                    }
                    KeyCode::Char('-') => {
                        self.apply_adjust(
                            &target,
                            axis,
                            -OFFSET_DELTAS[selected_delta].abs(),
                            data,
                        );
                    }
                    KeyCode::Char('+') | KeyCode::Char('=') => {
                        self.apply_adjust(&target, axis, OFFSET_DELTAS[selected_delta].abs(), data);
                    }
                    KeyCode::Enter | KeyCode::Char(' ') => {
                        self.apply_adjust(&target, axis, OFFSET_DELTAS[selected_delta], data);
                    }
                    KeyCode::Char(ch @ '1'..='4') => {
                        let magnitude = ch as usize - '1' as usize;
                        selected_delta = if OFFSET_DELTAS[selected_delta].is_sign_negative() {
                            magnitude
                        } else {
                            OFFSET_DELTAS.len() - 1 - magnitude
                        };
                    }
                    _ => {}
                }
                self.screen = Screen::AdjustEdit {
                    target,
                    axis,
                    selected_delta,
                };
                StepResult::Continue
            }
            Screen::Recenter { mut selected } => match key.code {
                KeyCode::Esc => {
                    self.screen = Screen::Dashboard;
                    StepResult::Continue
                }
                KeyCode::Up | KeyCode::Char('k') => {
                    selected = selected.saturating_sub(1);
                    self.screen = Screen::Recenter { selected };
                    StepResult::Continue
                }
                KeyCode::Down | KeyCode::Char('j') => {
                    selected = (selected + 1).min(1);
                    self.screen = Screen::Recenter { selected };
                    StepResult::Continue
                }
                KeyCode::Enter | KeyCode::Char(' ') => self.start_recenter(if selected == 0 {
                    SpaceKind::Stage
                } else {
                    SpaceKind::Local
                }),
                _ => StepResult::Continue,
            },
            Screen::Reset { mut selected } => {
                let targets = offset_targets(data);
                match key.code {
                    KeyCode::Esc => {
                        self.screen = Screen::Dashboard;
                    }
                    KeyCode::Up | KeyCode::Char('k') => {
                        selected = selected.saturating_sub(1);
                        self.screen = Screen::Reset { selected };
                    }
                    KeyCode::Down | KeyCode::Char('j') => {
                        selected = (selected + 1).min(targets.len().saturating_sub(1));
                        self.screen = Screen::Reset { selected };
                    }
                    KeyCode::Enter | KeyCode::Char(' ') => {
                        if let Some(target) = targets.get(selected) {
                            self.reset_target(target, data);
                            self.screen = Screen::Dashboard;
                        }
                    }
                    _ => {}
                }
                StepResult::Continue
            }
        }
    }

    fn handle_mouse(&mut self, mouse: MouseEvent, data: &mut CalibratorData<'_>) -> StepResult {
        match mouse.kind {
            MouseEventKind::ScrollUp
                if rect_contains(self.overview_area, mouse.column, mouse.row) =>
            {
                self.overview_scroll = self.overview_scroll.saturating_sub(3);
                return StepResult::Continue;
            }
            MouseEventKind::ScrollDown
                if rect_contains(self.overview_area, mouse.column, mouse.row) =>
            {
                self.overview_scroll = self.overview_scroll.saturating_add(3);
                return StepResult::Continue;
            }
            MouseEventKind::Down(MouseButton::Left) => {}
            _ => return StepResult::Continue,
        }

        let Some(action) = self
            .hitboxes
            .iter()
            .rev()
            .find(|hitbox| hitbox.contains(mouse.column, mouse.row))
            .map(|hitbox| hitbox.action.clone())
        else {
            return StepResult::Continue;
        };

        match action {
            MouseAction::Command(index) => {
                self.selected_command = index;
                self.activate_command(index, data)
            }
            MouseAction::CalibrateCycleDevice { source, delta } => {
                if let Screen::Calibrate(mut form) = self.screen.clone() {
                    let index = if source {
                        &mut form.source
                    } else {
                        &mut form.target
                    };
                    cycle_index(index, data.devices.len(), delta);
                    form.selected = if source { 0 } else { 1 };
                    self.screen = Screen::Calibrate(form);
                }
                StepResult::Continue
            }
            MouseAction::CalibrateSamples(delta) => {
                if let Screen::Calibrate(mut form) = self.screen.clone() {
                    let value = form.samples.parse::<i64>().unwrap_or(500);
                    form.samples = (value + i64::from(delta)).max(1).to_string();
                    form.selected = 2;
                    form.editing_samples = false;
                    self.screen = Screen::Calibrate(form);
                }
                StepResult::Continue
            }
            MouseAction::CalibrateToggle => {
                if let Screen::Calibrate(mut form) = self.screen.clone() {
                    form.continuous = !form.continuous;
                    form.selected = 3;
                    self.screen = Screen::Calibrate(form);
                }
                StepResult::Continue
            }
            MouseAction::CalibrateStart => {
                if let Screen::Calibrate(form) = self.screen.clone() {
                    self.start_calibration(&form, data)
                } else {
                    StepResult::Continue
                }
            }
            MouseAction::AdjustTarget(index) => {
                if let Some(target) = offset_targets(data).get(index).cloned() {
                    self.screen = Screen::AdjustEdit {
                        target,
                        axis: Axis::X,
                        selected_delta: 4,
                    };
                }
                StepResult::Continue
            }
            MouseAction::AdjustDelta { axis, delta } => {
                if let Screen::AdjustEdit { target, .. } = self.screen.clone() {
                    self.apply_adjust(&target, axis, delta, data);
                    let selected_delta = OFFSET_DELTAS
                        .iter()
                        .position(|candidate| *candidate == delta)
                        .unwrap_or(4);
                    self.screen = Screen::AdjustEdit {
                        target,
                        axis,
                        selected_delta,
                    };
                }
                StepResult::Continue
            }
            MouseAction::Recenter(space) => self.start_recenter(space),
            MouseAction::Reset(index) => {
                if let Some(target) = offset_targets(data).get(index) {
                    self.reset_target(target, data);
                    self.screen = Screen::Dashboard;
                }
                StepResult::Continue
            }
        }
    }

    fn activate_command(&mut self, command: usize, data: &mut CalibratorData<'_>) -> StepResult {
        self.status.clear();
        match command {
            0 => match continue_last_calibration(data) {
                Ok(result) => {
                    self.screen = Screen::Dashboard;
                    self.status = if matches!(&result, StepResult::Replace(_)) {
                        "Continuous calibration started.".into()
                    } else {
                        "Calibration applied.".into()
                    };
                    result
                }
                Err(message) => {
                    self.status = message;
                    StepResult::Continue
                }
            },
            1 => {
                if data.devices.len() < 2 {
                    self.status = "Calibration needs at least two devices.".into();
                    return StepResult::Continue;
                }
                let source = guess_source_device(data);
                let target = guess_target_device(data, source);
                self.screen = Screen::Calibrate(CalibrateForm {
                    source,
                    target,
                    samples: "500".into(),
                    continuous: false,
                    selected: 0,
                    editing_samples: false,
                });
                StepResult::Continue
            }
            2 => {
                self.screen = Screen::AdjustSelect { selected: 0 };
                StepResult::Continue
            }
            3 => {
                self.screen = Screen::Recenter { selected: 0 };
                StepResult::Continue
            }
            4 => {
                self.screen = Screen::Reset { selected: 0 };
                StepResult::Continue
            }
            _ => StepResult::Continue,
        }
    }

    fn adjust_calibrate_field(
        &self,
        form: &mut CalibrateForm,
        data: &CalibratorData<'_>,
        delta: isize,
    ) {
        match form.selected {
            0 => cycle_index(&mut form.source, data.devices.len(), delta),
            1 => cycle_index(&mut form.target, data.devices.len(), delta),
            2 => {
                let value = form.samples.parse::<i64>().unwrap_or(500);
                form.samples = (value + (delta as i64 * 50)).max(1).to_string();
                form.editing_samples = false;
            }
            3 => form.continuous = !form.continuous,
            _ => {}
        }
    }

    fn start_calibration(&mut self, form: &CalibrateForm, data: &CalibratorData<'_>) -> StepResult {
        let Some(source) = data.devices.get(form.source) else {
            self.status = "The source device is no longer available.".into();
            return StepResult::Continue;
        };
        let Some(target) = data.devices.get(form.target) else {
            self.status = "The target device is no longer available.".into();
            return StepResult::Continue;
        };
        if form.source == form.target {
            self.status = "Source and target must be different devices.".into();
            return StepResult::Continue;
        }
        if source.tracking_origin == target.tracking_origin {
            self.status = "Source and target must use different tracking origins.".into();
            return StepResult::Continue;
        }
        let Ok(samples) = form.samples.parse::<u32>() else {
            self.status = "Steps must be a positive whole number.".into();
            return StepResult::Continue;
        };
        if samples == 0 {
            self.status = "Steps must be greater than zero.".into();
            return StepResult::Continue;
        }

        self.screen = Screen::Dashboard;
        self.status = "Calibration started.".into();
        StepResult::Replace(Box::new(SampledMethod::new(
            form.source,
            form.target,
            form.continuous,
            samples,
            "last".into(),
        )))
    }

    fn start_recenter(&mut self, space: SpaceKind) -> StepResult {
        let height = None;
        match RecenterMethod::new(space.argument(), &height) {
            Ok(method) => {
                self.screen = Screen::Dashboard;
                self.status = format!("Recentering {}.", space.name());
                StepResult::Replace(Box::new(method))
            }
            Err(error) => {
                self.status = format!("Could not recenter {}: {error}", space.name());
                StepResult::Continue
            }
        }
    }

    fn apply_adjust(
        &mut self,
        target: &AdjustTarget,
        axis: Axis,
        delta: f64,
        data: &CalibratorData<'_>,
    ) {
        let result = (|| -> std::result::Result<(), String> {
            let mut offset = get_target_offset(target, data)?;
            match axis {
                Axis::X => offset.origin.x += delta,
                Axis::Y => offset.origin.y += delta,
                Axis::Z => offset.origin.z += delta,
                Axis::Pitch | Axis::Yaw | Axis::Roll => {
                    let radians = delta.to_radians();
                    let rotation = match axis {
                        Axis::Pitch => Rotation3::from_axis_angle(&Vector3::x_axis(), radians),
                        Axis::Yaw => Rotation3::from_axis_angle(&Vector3::y_axis(), radians),
                        Axis::Roll => Rotation3::from_axis_angle(&Vector3::z_axis(), radians),
                        _ => unreachable!(),
                    };
                    offset.basis = rotation * offset.basis;
                }
            }
            set_target_offset(target, offset, data)
        })();

        self.status = match result {
            Ok(()) => format!(
                "{} {:+.2} {} applied to {}.",
                axis.name(),
                delta,
                axis.unit(),
                target.label()
            ),
            Err(message) => message,
        };
    }

    fn reset_target(&mut self, target: &AdjustTarget, data: &CalibratorData<'_>) {
        self.status = match set_target_offset(target, TransformD::default(), data) {
            Ok(()) => format!("{} has been reset.", target.label()),
            Err(message) => message,
        };
    }
}

impl Default for Tui {
    fn default() -> Self {
        Self::new(TuiLogBuffer::new())
    }
}

impl Drop for Tui {
    fn drop(&mut self) {
        let _ = self.stop_terminal();
    }
}

impl Tui {
    pub fn init(&mut self) -> Result<()> {
        self.start_terminal()
    }

    pub fn step(
        &mut self,
        data: &mut CalibratorData<'_>,
        calibrator_status: Option<&CalibratorStatus>,
    ) -> Result<StepResult> {
        self.draw(data, calibrator_status)?;

        while event::poll(Duration::ZERO).context("Unable to poll terminal input")? {
            let result = match event::read().context("Unable to read terminal input")? {
                Event::Key(key) => self.handle_key(key, data),
                Event::Mouse(mouse) => self.handle_mouse(mouse, data),
                Event::Resize(_, _) => {
                    if let Some(terminal) = self.terminal.as_mut() {
                        terminal
                            .autoresize()
                            .context("Unable to resize the terminal")?;
                    }
                    StepResult::Continue
                }
                _ => StepResult::Continue,
            };

            if !matches!(result, StepResult::Continue) {
                return Ok(result);
            }
        }

        Ok(StepResult::Continue)
    }

    pub fn finish(&mut self) -> Result<()> {
        self.stop_terminal()
    }

    pub fn set_status(&mut self, status: impl Into<String>) {
        self.status = status.into();
    }
}

fn continue_last_calibration(
    data: &mut CalibratorData<'_>,
) -> std::result::Result<StepResult, String> {
    let last = data.load_calibration("last").map_err(|error| {
        format!(
            "Could not load the 'last' calibration. Calibrate first, or check the saved profile: \
             {error}"
        )
    })?;

    match last.offset_type {
        OffsetType::TrackingOrigin => {
            let source = data
                .tracking_origins
                .iter()
                .find(|origin| origin.name == last.src)
                .map(|origin| {
                    origin
                        .get_offset()
                        .map(|pose| -> TransformD { pose.into() })
                        .map_err(|error| error.to_string())
                })
                .transpose()?
                .unwrap_or_default();

            let Some(target) = data
                .tracking_origins
                .iter()
                .find(|origin| origin.name == last.dst)
            else {
                return Err(format!("No such tracking origin: {}", last.dst));
            };

            target
                .set_offset((last.offset * source).into())
                .map_err(|error| format!("Could not apply calibration: {error}"))?;
            Ok(StepResult::Continue)
        }
        OffsetType::Device => {
            let source = data
                .devices
                .iter()
                .position(|device| device.serial == last.src)
                .ok_or_else(|| format!("No such device: {}", last.src))?;
            let target = data
                .devices
                .iter()
                .position(|device| device.serial == last.dst)
                .ok_or_else(|| format!("No such device: {}", last.dst))?;

            Ok(StepResult::Replace(Box::new(OffsetMethod::new_internal(
                source,
                target,
                last.offset,
                0.02,
            ))))
        }
    }
}

fn get_target_offset(
    target: &AdjustTarget,
    data: &CalibratorData<'_>,
) -> std::result::Result<TransformD, String> {
    match target {
        AdjustTarget::Space(space) => data
            .monado
            .get_reference_space_offset(space.reference_type())
            .map(Into::into)
            .map_err(|error| format!("Could not read the {} offset: {error}", target.label())),
        AdjustTarget::TrackingOrigin { id, .. } => data
            .tracking_origins
            .iter()
            .find(|origin| origin.id == *id)
            .ok_or_else(|| format!("{} is no longer available.", target.label()))?
            .get_offset()
            .map(Into::into)
            .map_err(|error| format!("Could not read the {} offset: {error}", target.label())),
    }
}

fn set_target_offset(
    target: &AdjustTarget,
    offset: TransformD,
    data: &CalibratorData<'_>,
) -> std::result::Result<(), String> {
    match target {
        AdjustTarget::Space(space) => data
            .monado
            .set_reference_space_offset(space.reference_type(), offset.into())
            .map_err(|error| format!("Could not update the {} offset: {error}", target.label())),
        AdjustTarget::TrackingOrigin { id, .. } => data
            .tracking_origins
            .iter()
            .find(|origin| origin.id == *id)
            .ok_or_else(|| format!("{} is no longer available.", target.label()))?
            .set_offset(offset.into())
            .map_err(|error| format!("Could not update the {} offset: {error}", target.label())),
    }
}

fn guess_source_device(data: &CalibratorData<'_>) -> usize {
    data.devices
        .iter()
        .position(|device| {
            let identity = format!("{} {}", device.serial, device.inner.name).to_lowercase();
            identity.contains("hmd") || identity.contains("head") || identity.contains("display")
        })
        .unwrap_or(0)
}

fn guess_target_device(data: &CalibratorData<'_>, source: usize) -> usize {
    let source_origin = data.devices[source].tracking_origin;
    data.devices
        .iter()
        .position(|device| device.tracking_origin != source_origin)
        .unwrap_or(if source == 0 { 1 } else { 0 })
}

fn cycle_index(index: &mut usize, len: usize, delta: isize) {
    if len == 0 {
        *index = 0;
        return;
    }
    *index = (*index as isize + delta).rem_euclid(len as isize) as usize;
}

fn offset_targets(data: &CalibratorData<'_>) -> Vec<AdjustTarget> {
    let mut targets = vec![
        AdjustTarget::Space(SpaceKind::Stage),
        AdjustTarget::Space(SpaceKind::Local),
    ];
    targets.extend(
        data.tracking_origins
            .iter()
            .map(|origin| AdjustTarget::TrackingOrigin {
                id: origin.id,
                name: origin.name.clone(),
            }),
    );
    targets
}

#[allow(clippy::too_many_arguments)]
fn draw_ui(
    frame: &mut Frame<'_>,
    data: &CalibratorData<'_>,
    screen: &Screen,
    selected_command: usize,
    overview_scroll: u16,
    status: &str,
    calibrator_status: Option<&CalibratorStatus>,
    spinner_char: char,
    logs: &TuiLogBuffer,
    hitboxes: &mut Vec<Hitbox>,
    overview_area: &mut Rect,
) {
    let outer = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(8), Constraint::Length(2)])
        .split(frame.area());
    let panels = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(60), Constraint::Percentage(40)])
        .split(outer[0]);

    *overview_area = panels[0];
    draw_overview(frame, panels[0], data, overview_scroll);

    let right_panels = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(7), Constraint::Length(5)])
        .split(panels[1]);

    match screen {
        Screen::Dashboard => {
            draw_commands(frame, right_panels[0], selected_command, hitboxes);
        }
        Screen::Calibrate(form) => {
            draw_calibrate(frame, right_panels[0], data, form, hitboxes);
        }
        Screen::AdjustSelect { selected } => {
            draw_target_list(
                frame,
                right_panels[0],
                " Adjust: select target ",
                &offset_targets(data),
                *selected,
                true,
                hitboxes,
            );
        }
        Screen::AdjustEdit {
            target,
            axis,
            selected_delta,
        } => {
            draw_offset_editor(
                frame,
                right_panels[0],
                data,
                target,
                *axis,
                *selected_delta,
                hitboxes,
            );
        }
        Screen::Recenter { selected } => {
            draw_recenter(frame, right_panels[0], *selected, hitboxes);
        }
        Screen::Reset { selected } => {
            draw_target_list(
                frame,
                right_panels[0],
                " Reset: select target ",
                &offset_targets(data),
                *selected,
                false,
                hitboxes,
            );
        }
    }
    draw_logs(frame, right_panels[1], logs);

    let help = match screen {
        Screen::Dashboard => "  ↑/↓ select  Enter run  PgUp/PgDn scroll  q quit",
        Screen::Calibrate(_) => "  ↑/↓ field  ←/→ change  type steps  Enter activate  Esc back",
        Screen::AdjustSelect { .. } | Screen::Recenter { .. } | Screen::Reset { .. } => {
            "  ↑/↓ select  Enter confirm  Esc back"
        }
        Screen::AdjustEdit { .. } => {
            "  ↑/↓ axis  ←/→ button  Enter activate  +/- apply sign  1..4 magnitude  Esc targets"
        }
    };
    let mut footer = Vec::new();
    footer.push(Span::styled(help, Style::default().fg(Color::Gray)));
    if let Some(calibrator_status) = calibrator_status {
        let text = match calibrator_status {
            CalibratorStatus::Spinner { message } => format!("  |  {spinner_char} {message}"),
            CalibratorStatus::Progress {
                current,
                max,
                message,
            } => format!("  |  [{current}/{max}] {message}"),
        };
        footer.push(Span::styled(text, Style::default().fg(Color::LightCyan)));
        footer.push(Span::raw("  "));
    } else if !status.is_empty() {
        footer.push(Span::styled(
            format!("{status}  "),
            Style::default().fg(
                if status.starts_with("Could not")
                    || status.starts_with("No such")
                    || status.contains("must")
                    || status.contains("needs")
                {
                    Color::LightRed
                } else {
                    Color::LightGreen
                },
            ),
        ));
    }
    frame.render_widget(
        Paragraph::new(Line::from(footer))
            .block(Block::default().borders(Borders::TOP))
            .alignment(Alignment::Left),
        outer[1],
    );
}

fn draw_overview(frame: &mut Frame<'_>, area: Rect, data: &CalibratorData<'_>, scroll: u16) {
    let block = Block::default()
        .title(" Monado Universe ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Blue));
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let lines = overview_lines(data);
    let max_scroll = lines.len().saturating_sub(inner.height as usize) as u16;
    frame.render_widget(
        Paragraph::new(lines).scroll((scroll.min(max_scroll), 0)),
        inner,
    );
}

fn overview_lines(data: &CalibratorData<'_>) -> Vec<Line<'static>> {
    let mut lines = Vec::new();
    lines.push(section_line("Spaces"));
    for space in [SpaceKind::Stage, SpaceKind::Local] {
        lines.push(Line::from(Span::styled(
            format!("  {}", space.name()),
            Style::default()
                .fg(Color::LightBlue)
                .add_modifier(Modifier::BOLD),
        )));
        match data
            .monado
            .get_reference_space_offset(space.reference_type())
        {
            Ok(pose) => {
                let (pitch, yaw, roll) =
                    UnitQuaternion::from_quaternion(Quaternion::from(pose.orientation))
                        .euler_angles();
                lines.push(offset_line(
                    "    ",
                    pose.position.x as f64,
                    pose.position.y as f64,
                    pose.position.z as f64,
                    pitch as f64,
                    yaw as f64,
                    roll as f64,
                ));
            }
            Err(error) => lines.push(unavailable_line("    ", &error.to_string())),
        }
    }

    lines.push(Line::default());
    lines.push(section_line("Tracking origins"));

    let mut origin_ids = data
        .tracking_origins
        .iter()
        .map(|origin| origin.id)
        .collect::<Vec<_>>();
    for device in &data.devices {
        if !origin_ids.contains(&device.tracking_origin) {
            origin_ids.push(device.tracking_origin);
        }
    }

    if origin_ids.is_empty() {
        lines.push(Line::from(Span::styled(
            "  No tracking origins",
            Style::default().fg(Color::DarkGray),
        )));
    }

    for (origin_index, origin_id) in origin_ids.iter().copied().enumerate() {
        let last_origin = origin_index + 1 == origin_ids.len();
        let origin_branch = if last_origin {
            "  └─ "
        } else {
            "  ├─ "
        };
        let origin_child_prefix = if last_origin { "     " } else { "  │  " };
        let origin = data
            .tracking_origins
            .iter()
            .find(|origin| origin.id == origin_id);
        let name = if let Some(origin) = origin {
            if origin.name.is_empty() {
                "Unknown tracking origin"
            } else {
                &origin.name
            }
        } else {
            "Unlisted tracking origin"
        };
        lines.push(Line::from(Span::styled(
            format!("{origin_branch}[{origin_id}] {name}"),
            Style::default()
                .fg(Color::LightBlue)
                .add_modifier(Modifier::BOLD),
        )));

        if let Some(origin) = origin {
            match origin.get_offset() {
                Ok(pose) => {
                    let (roll, pitch, yaw) =
                        UnitQuaternion::from_quaternion(Quaternion::from(pose.orientation))
                            .euler_angles();
                    lines.push(offset_line(
                        origin_child_prefix,
                        pose.position.x as f64,
                        pose.position.y as f64,
                        pose.position.z as f64,
                        pitch as f64,
                        yaw as f64,
                        roll as f64,
                    ));
                }
                Err(error) => lines.push(unavailable_line(origin_child_prefix, &error.to_string())),
            }
        }

        let devices = data
            .devices
            .iter()
            .filter(|device| device.tracking_origin == origin_id)
            .collect::<Vec<_>>();
        for (device_index, device) in devices.iter().enumerate() {
            let last_device = device_index + 1 == devices.len();
            let device_branch = if last_device { "└─ " } else { "├─ " };
            let device_child_prefix = if last_device { "   " } else { "│  " };
            let display_name = if device.inner.name.is_empty() || device.inner.name == device.serial
            {
                format!("[{}] {}", device.index, device.serial)
            } else {
                format!(
                    "[{}] {} ({})",
                    device.index, device.inner.name, device.serial
                )
            };

            let mut device_line = vec![Span::styled(
                format!("{origin_child_prefix}{device_branch}{display_name}"),
                Style::default()
                    .fg(Color::LightYellow)
                    .add_modifier(Modifier::BOLD),
            )];
            if let Ok(battery) = device.inner.battery_status() {
                if battery.present {
                    let color = if battery.charging {
                        Color::LightBlue
                    } else if battery.charge > 0.4 {
                        Color::LightGreen
                    } else if battery.charge > 0.2 {
                        Color::Yellow
                    } else {
                        Color::LightRed
                    };
                    let symbol = if battery.charging { "⚡" } else { "🔋" };
                    device_line.push(Span::styled(
                        format!("  {symbol}{:.0}%", battery.charge * 100.0),
                        Style::default().fg(color),
                    ));
                }
            }
            lines.push(Line::from(device_line));

            let detail_prefix = format!("{origin_child_prefix}{device_child_prefix}");
            match device.space.relate(&data.stage, data.now) {
                Ok((_, velocity)) => lines.push(speed_line(
                    &detail_prefix,
                    velocity.linear_velocity,
                    velocity.angular_velocity,
                    velocity
                        .velocity_flags
                        .intersects(SpaceVelocityFlags::LINEAR_VALID),
                    velocity
                        .velocity_flags
                        .intersects(SpaceVelocityFlags::ANGULAR_VALID),
                )),
                Err(error) => lines.push(unavailable_line(&detail_prefix, &error.to_string())),
            }
        }
    }

    if data.devices.is_empty() {
        lines.push(Line::from(Span::styled(
            "  No devices",
            Style::default().fg(Color::DarkGray),
        )));
    }

    lines
}

fn section_line(title: &str) -> Line<'static> {
    Line::from(Span::styled(
        title.to_owned(),
        Style::default()
            .fg(Color::Cyan)
            .add_modifier(Modifier::BOLD | Modifier::UNDERLINED),
    ))
}

#[allow(clippy::too_many_arguments)]
fn offset_line(
    prefix: &str,
    x: f64,
    y: f64,
    z: f64,
    pitch: f64,
    yaw: f64,
    roll: f64,
) -> Line<'static> {
    Line::from(Span::styled(
        format!(
            "{prefix}X {x:+.3}  Y {y:+.3}  Z {z:+.3} m  │  P {:+.2}°  Y {:+.2}°  R {:+.2}°",
            pitch.to_degrees(),
            yaw.to_degrees(),
            roll.to_degrees()
        ),
        Style::default().fg(Color::Gray),
    ))
}

fn speed_line(
    prefix: &str,
    linear_velocity: impl Into<mint::Vector3<f32>>,
    angular_velocity: impl Into<mint::Vector3<f32>>,
    linear_valid: bool,
    angular_valid: bool,
) -> Line<'static> {
    let linear: mint::Vector3<f32> = linear_velocity.into();
    let angular: mint::Vector3<f32> = angular_velocity.into();
    let linear: Vector3<f32> = linear.into();
    let angular: Vector3<f32> = angular.into();
    let speed = linear.norm();
    let spin = angular.norm();
    let speed_ticks = (speed * TICKER_SIZE as f32).clamp(0.0, TICKER_SIZE as f32) as usize;
    let spin_ticks = (spin / std::f32::consts::PI * 2.0).clamp(0.0, TICKER_SIZE as f32) as usize;

    Line::from(vec![
        Span::raw(prefix.to_owned()),
        Span::styled(
            format!(
                "Speed [{}{}] {speed:.2} m/s",
                "█".repeat(speed_ticks),
                "·".repeat(TICKER_SIZE - speed_ticks)
            ),
            Style::default().fg(if linear_valid {
                Color::LightGreen
            } else {
                Color::DarkGray
            }),
        ),
        Span::raw("  "),
        Span::styled(
            format!(
                "Spin [{}{}] {spin:.2} rad/s",
                "█".repeat(spin_ticks),
                "·".repeat(TICKER_SIZE - spin_ticks)
            ),
            Style::default().fg(if angular_valid {
                Color::LightGreen
            } else {
                Color::DarkGray
            }),
        ),
    ])
}

fn unavailable_line(prefix: &str, error: &str) -> Line<'static> {
    Line::from(Span::styled(
        format!("{prefix}unavailable: {error}"),
        Style::default().fg(Color::DarkGray),
    ))
}

fn draw_commands(frame: &mut Frame<'_>, area: Rect, selected: usize, hitboxes: &mut Vec<Hitbox>) {
    let block = Block::default()
        .title(" Commands ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Blue));
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let items = COMMANDS
        .iter()
        .map(|command| ListItem::new(Line::from(format!("  {command}"))))
        .collect::<Vec<_>>();
    let mut state = ListState::default();
    state.select(Some(selected));
    frame.render_stateful_widget(
        List::new(items).highlight_symbol("▶ ").highlight_style(
            Style::default()
                .fg(Color::Black)
                .bg(Color::LightCyan)
                .add_modifier(Modifier::BOLD),
        ),
        inner,
        &mut state,
    );

    for (index, _) in COMMANDS.iter().enumerate() {
        if index < inner.height as usize {
            hitboxes.push(Hitbox {
                rect: Rect::new(inner.x, inner.y + index as u16, inner.width, 1),
                action: MouseAction::Command(index),
            });
        }
    }
}

fn draw_logs(frame: &mut Frame<'_>, area: Rect, logs: &TuiLogBuffer) {
    let block = Block::default()
        .title(" Logs ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Blue));
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let lines = logs
        .recent(inner.height as usize)
        .into_iter()
        .map(|line| {
            let color = match line.level {
                Level::Error => Color::LightRed,
                Level::Warn => Color::LightYellow,
                Level::Info => Color::LightCyan,
                Level::Debug | Level::Trace => Color::DarkGray,
            };
            Line::from(vec![
                Span::styled(
                    format!("[{:<5}] ", line.level),
                    Style::default().fg(color).add_modifier(Modifier::BOLD),
                ),
                Span::raw(line.message),
            ])
        })
        .collect::<Vec<_>>();
    frame.render_widget(Paragraph::new(lines), inner);
}

fn draw_calibrate(
    frame: &mut Frame<'_>,
    area: Rect,
    data: &CalibratorData<'_>,
    form: &CalibrateForm,
    hitboxes: &mut Vec<Hitbox>,
) {
    let block = Block::default()
        .title(" Calibrate ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Blue));
    let inner = block.inner(area);
    frame.render_widget(block, area);
    frame.render_widget(
        Paragraph::new("Move both devices together while samples are collected.")
            .style(Style::default().fg(Color::Gray))
            .wrap(ratatui::widgets::Wrap { trim: true }),
        bounded_rect(inner, 0, 0, inner.width, 2),
    );

    let source = device_label(data, form.source);
    let target = device_label(data, form.target);
    draw_selector(
        frame,
        bounded_rect(inner, 0, 3, inner.width, 1),
        "Source",
        &source,
        form.selected == 0,
        MouseAction::CalibrateCycleDevice {
            source: true,
            delta: -1,
        },
        MouseAction::CalibrateCycleDevice {
            source: true,
            delta: 1,
        },
        hitboxes,
    );
    draw_selector(
        frame,
        bounded_rect(inner, 0, 5, inner.width, 1),
        "Target",
        &target,
        form.selected == 1,
        MouseAction::CalibrateCycleDevice {
            source: false,
            delta: -1,
        },
        MouseAction::CalibrateCycleDevice {
            source: false,
            delta: 1,
        },
        hitboxes,
    );
    draw_selector(
        frame,
        bounded_rect(inner, 0, 7, inner.width, 1),
        "Steps",
        if form.samples.is_empty() {
            " "
        } else {
            &form.samples
        },
        form.selected == 2,
        MouseAction::CalibrateSamples(-50),
        MouseAction::CalibrateSamples(50),
        hitboxes,
    );
    let checkbox = bounded_rect(inner, 0, 9, inner.width, 1);
    frame.render_widget(
        Paragraph::new(format!(
            "{} Continuous mode (tracker on HMD)",
            if form.continuous { "[x]" } else { "[ ]" }
        ))
        .style(field_style(form.selected == 3)),
        checkbox,
    );
    hitboxes.push(Hitbox {
        rect: checkbox,
        action: MouseAction::CalibrateToggle,
    });

    draw_button(
        frame,
        bounded_rect(inner, 0, 12, inner.width, 3),
        "Start",
        form.selected == 4,
        MouseAction::CalibrateStart,
        hitboxes,
    );
}

#[allow(clippy::too_many_arguments)]
fn draw_selector(
    frame: &mut Frame<'_>,
    area: Rect,
    label: &str,
    value: &str,
    selected: bool,
    previous: MouseAction,
    next: MouseAction,
    hitboxes: &mut Vec<Hitbox>,
) {
    if area.width == 0 || area.height == 0 {
        return;
    }
    let chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Length(8),
            Constraint::Length(3),
            Constraint::Min(1),
            Constraint::Length(3),
        ])
        .split(area);
    frame.render_widget(
        Paragraph::new(label).style(field_style(selected)),
        chunks[0],
    );
    frame.render_widget(
        Paragraph::new("<")
            .alignment(Alignment::Center)
            .style(button_style(false)),
        chunks[1],
    );
    frame.render_widget(
        Paragraph::new(value)
            .alignment(Alignment::Center)
            .style(field_style(selected)),
        chunks[2],
    );
    frame.render_widget(
        Paragraph::new(">")
            .alignment(Alignment::Center)
            .style(button_style(false)),
        chunks[3],
    );
    hitboxes.push(Hitbox {
        rect: chunks[1],
        action: previous,
    });
    hitboxes.push(Hitbox {
        rect: chunks[3],
        action: next,
    });
}

fn draw_target_list(
    frame: &mut Frame<'_>,
    area: Rect,
    title: &str,
    targets: &[AdjustTarget],
    selected: usize,
    edit_after_select: bool,
    hitboxes: &mut Vec<Hitbox>,
) {
    let block = Block::default()
        .title(title)
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Blue));
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let items = targets
        .iter()
        .map(|target| ListItem::new(format!("  {}", target.label())))
        .collect::<Vec<_>>();
    let mut state = ListState::default();
    state.select(Some(selected.min(targets.len().saturating_sub(1))));
    frame.render_stateful_widget(
        List::new(items).highlight_symbol("▶ ").highlight_style(
            Style::default()
                .fg(Color::Black)
                .bg(Color::LightCyan)
                .add_modifier(Modifier::BOLD),
        ),
        inner,
        &mut state,
    );

    for (index, _) in targets.iter().enumerate().take(inner.height as usize) {
        hitboxes.push(Hitbox {
            rect: Rect::new(inner.x, inner.y + index as u16, inner.width, 1),
            action: if edit_after_select {
                MouseAction::AdjustTarget(index)
            } else {
                MouseAction::Reset(index)
            },
        });
    }
}

fn draw_offset_editor(
    frame: &mut Frame<'_>,
    area: Rect,
    data: &CalibratorData<'_>,
    target: &AdjustTarget,
    selected_axis: Axis,
    selected_delta: usize,
    hitboxes: &mut Vec<Hitbox>,
) {
    let block = Block::default()
        .title(format!(" Offset: {} ", target.label()))
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Blue));
    let inner = block.inner(area);
    frame.render_widget(block, area);
    let sections = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(12), Constraint::Length(3)])
        .split(inner);
    let controls = sections[0];

    let values = get_target_offset(target, data).ok().map(|offset| {
        let (pitch, yaw, roll) = offset.basis.euler_angles();
        [
            offset.origin.x,
            offset.origin.y,
            offset.origin.z,
            pitch.to_degrees(),
            yaw.to_degrees(),
            roll.to_degrees(),
        ]
    });

    for (index, axis) in Axis::ALL.iter().copied().enumerate() {
        let y = index as u16 * 2;
        let value = values.map(|values| values[index]);
        let label = match value {
            Some(value) => format!("{}: {value:+.3} {}", axis.name(), axis.unit()),
            None => format!("{}: unavailable", axis.name()),
        };
        frame.render_widget(
            Paragraph::new(label).style(field_style(axis == selected_axis)),
            bounded_rect(controls, 0, y, controls.width, 1),
        );

        let button_area = bounded_rect(controls, 0, y + 1, controls.width, 1);
        let labels = ["-10", "-1", "-.1", "-.01", "+.01", "+.1", "+1", "+10"];
        let buttons = split_evenly(button_area, OFFSET_DELTAS.len());
        for ((button, delta), label) in buttons
            .into_iter()
            .zip(OFFSET_DELTAS)
            .zip(labels)
        {
            let keyboard_selected = axis == selected_axis && delta == OFFSET_DELTAS[selected_delta];
            frame.render_widget(
                Paragraph::new(label)
                    .alignment(Alignment::Center)
                    .style(button_style(keyboard_selected)),
                button,
            );
            hitboxes.push(Hitbox {
                rect: button,
                action: MouseAction::AdjustDelta { axis, delta },
            });
        }
    }
}

fn draw_recenter(frame: &mut Frame<'_>, area: Rect, selected: usize, hitboxes: &mut Vec<Hitbox>) {
    let block = Block::default()
        .title(" Recenter: select space ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Blue));
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let spaces = [SpaceKind::Stage, SpaceKind::Local];
    let items = spaces
        .iter()
        .map(|space| ListItem::new(format!("  {}", space.name())))
        .collect::<Vec<_>>();
    let mut state = ListState::default();
    state.select(Some(selected.min(1)));
    frame.render_stateful_widget(
        List::new(items).highlight_symbol("▶ ").highlight_style(
            Style::default()
                .fg(Color::Black)
                .bg(Color::LightCyan)
                .add_modifier(Modifier::BOLD),
        ),
        inner,
        &mut state,
    );
    for (index, space) in spaces.iter().copied().enumerate() {
        hitboxes.push(Hitbox {
            rect: Rect::new(inner.x, inner.y + index as u16, inner.width, 1),
            action: MouseAction::Recenter(space),
        });
    }
}

fn draw_button(
    frame: &mut Frame<'_>,
    area: Rect,
    label: &str,
    selected: bool,
    action: MouseAction,
    hitboxes: &mut Vec<Hitbox>,
) {
    frame.render_widget(Clear, area);
    frame.render_widget(
        Paragraph::new(label)
            .alignment(Alignment::Center)
            .style(button_style(selected))
            .block(Block::default().borders(Borders::ALL)),
        area,
    );
    hitboxes.push(Hitbox { rect: area, action });
}

fn field_style(selected: bool) -> Style {
    if selected {
        Style::default()
            .fg(Color::Black)
            .bg(Color::LightCyan)
            .add_modifier(Modifier::BOLD)
    } else {
        Style::default().fg(Color::White)
    }
}

fn button_style(selected: bool) -> Style {
    if selected {
        Style::default()
            .fg(Color::Black)
            .bg(Color::LightYellow)
            .add_modifier(Modifier::BOLD)
    } else {
        Style::default().fg(Color::Cyan)
    }
}

fn device_label(data: &CalibratorData<'_>, index: usize) -> String {
    let Some(device) = data.devices.get(index) else {
        return "Unavailable".into();
    };
    if device.inner.name.is_empty() || device.inner.name == device.serial {
        format!("[{}] {}", device.index, device.serial)
    } else {
        format!("[{}] {}", device.index, device.inner.name)
    }
}

fn split_evenly(area: Rect, count: usize) -> Vec<Rect> {
    if count == 0 || area.width == 0 || area.height == 0 {
        return Vec::new();
    }
    let count = count.min(area.width as usize);
    let base = area.width / count as u16;
    let remainder = area.width % count as u16;
    let mut x = area.x;
    (0..count)
        .map(|index| {
            let width = base + u16::from(index < remainder as usize);
            let rect = Rect::new(x, area.y, width, area.height);
            x = x.saturating_add(width);
            rect
        })
        .collect()
}

fn bounded_rect(area: Rect, x: u16, y: u16, width: u16, height: u16) -> Rect {
    if x >= area.width || y >= area.height {
        return Rect::new(area.x, area.y, 0, 0);
    }
    Rect::new(
        area.x + x,
        area.y + y,
        width.min(area.width - x),
        height.min(area.height - y),
    )
}

fn rect_contains(rect: Rect, column: u16, row: u16) -> bool {
    column >= rect.x
        && column < rect.x.saturating_add(rect.width)
        && row >= rect.y
        && row < rect.y.saturating_add(rect.height)
}
