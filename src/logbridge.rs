// https://github.com/djugei/indicatif-log-bridge
// Copyright © 2025 djugei
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

use indicatif::MultiProgress;
use log::Log;

/// Wraps a MultiProgress and a Log implementor,
/// calling .suspend on the MultiProgress while writing the log message,
/// thereby preventing progress bars and logs from getting mixed up.
///
/// You simply have to add every ProgressBar to the passed MultiProgress.
pub struct LogWrapper<L: Log> {
    bar: MultiProgress,
    log: L,
}

impl<L: Log + 'static> LogWrapper<L> {
    pub fn new(bar: MultiProgress, log: L) -> Self {
        Self { bar, log }
    }

    /// Installs this as the global logger.
    ///
    /// Tries to find the correct argument to log::set_max_level
    /// by reading the logger configuration,
    /// you may want to set it manually though.
    /// For more details read the [known issues](index.html#wrong-global-log-level).
    pub fn try_init(self) -> Result<(), log::SetLoggerError> {
        use log::LevelFilter::*;
        let levels = [Off, Error, Warn, Info, Debug, Trace];

        for level_filter in levels.iter().rev() {
            let level = if let Some(level) = level_filter.to_level() {
                level
            } else {
                // off is the last level, just do nothing in that case
                continue;
            };
            let meta = log::Metadata::builder().level(level).build();
            if self.enabled(&meta) {
                log::set_max_level(*level_filter);
                break;
            }
        }

        log::set_boxed_logger(Box::new(self))
    }
}

impl<L: Log> Log for LogWrapper<L> {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        self.log.enabled(metadata)
    }

    fn log(&self, record: &log::Record) {
        // do an early check for enabled to not cause unnescesary suspends
        if self.log.enabled(record.metadata()) {
            self.bar.suspend(|| self.log.log(record))
        }
    }

    fn flush(&self) {
        self.log.flush()
    }
}
