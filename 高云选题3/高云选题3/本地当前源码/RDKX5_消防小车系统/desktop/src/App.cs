using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Net.Http;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Web.Script.Serialization;
using System.Windows.Forms;
using Microsoft.Web.WebView2.Core;
using Microsoft.Web.WebView2.WinForms;

namespace RescueDesktop
{
    static class Program
    {
        [DllImport("user32.dll")] static extern bool SetProcessDpiAwarenessContext(IntPtr value);
        [STAThread]
        static void Main()
        {
            try { SetProcessDpiAwarenessContext(new IntPtr(-4)); } catch (EntryPointNotFoundException) { }
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.ThreadException += (sender, e) => MessageBox.Show(e.Exception.Message, "控制台提示", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            Application.Run(new PresentationForm());
        }
    }

    sealed class PresentationForm : Form
    {
        [DllImport("dwmapi.dll")] static extern int DwmSetWindowAttribute(IntPtr hwnd, int attribute, ref int value, int size);
        readonly WebView2 view = new WebView2();
        readonly ComboBox address = new ComboBox();
        readonly Label connection = new Label();
        readonly Panel cover = new Panel();
        readonly Label coverTitle = new Label();
        readonly Label coverBody = new Label();
        readonly Button liveButton;
        readonly Button reloadButton;
        readonly string dataDirectory = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "TreeBird", "RescuePresentation");
        readonly HttpClient http = new HttpClient(new HttpClientHandler { UseProxy = false, AllowAutoRedirect = false }) { Timeout = TimeSpan.FromSeconds(5) };
        readonly System.Windows.Forms.Timer reconnectTimer = new System.Windows.Forms.Timer { Interval = 3000 };
        CancellationTokenSource connectionCancellation;
        Uri allowedOrigin;
        bool closing;
        bool initializing;
        bool ready;
        bool connectionInFlight;
        bool updatingAddress;
        int navigationGeneration;
        int pageGeneration;
        ulong pageNavigationId;

        public PresentationForm()
        {
            // Build the complete 96-DPI control tree before WinForms performs its
            // first scale pass. An early ClientSize/layout pass can otherwise
            // consume the DPI factor before the toolbar controls are added.
            SuspendLayout();
            Text = "消防救援控制台";
            Font = new Font("Microsoft YaHei UI", 10.5F);
            BackColor = Color.FromArgb(11, 13, 16);
            MinimumSize = new Size(1024, 640);
            ClientSize = new Size(1360, 840);
            StartPosition = FormStartPosition.CenterScreen;
            WindowState = FormWindowState.Maximized;
            try { Icon = Icon.ExtractAssociatedIcon(Application.ExecutablePath); } catch { }

            var header = new TableLayoutPanel { Dock = DockStyle.Top, Height = 44, Padding = new Padding(12, 6, 12, 6), BackColor = Color.FromArgb(11, 13, 16), ColumnCount = 7, RowCount = 1 };
            header.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 126));
            header.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 246));
            header.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 102));
            header.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 76));
            header.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            header.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 180));
            header.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 64));
            var brand = new Label { Text = "RESCUE / X5", Dock = DockStyle.Fill, TextAlign = ContentAlignment.MiddleLeft, ForeColor = Color.FromArgb(230, 233, 237), Font = new Font("Bahnschrift", 10.5F, FontStyle.Bold), Margin = new Padding(0, 0, 6, 0) };
            header.Controls.Add(brand, 0, 0);
            address.Dock = DockStyle.Fill;
            address.Margin = new Padding(0, 2, 0, 0);
            address.Font = new Font("Consolas", 10.5F);
            address.FlatStyle = FlatStyle.Flat;
            address.BackColor = Color.FromArgb(7, 8, 9);
            address.ForeColor = Color.FromArgb(230, 233, 237);
            address.Items.Add("192.168.127.10:8090");
            address.Text = ReadAddress();
            address.AccessibleName = "X5 局域网 IP 和端口";
            header.Controls.Add(address, 1, 0);
            liveButton = MakeButton("连接 X5");
            liveButton.BackColor = Color.FromArgb(10, 38, 48);
            liveButton.ForeColor = Color.FromArgb(59, 196, 230);
            liveButton.FlatAppearance.BorderColor = Color.FromArgb(59, 196, 230);
            liveButton.FlatAppearance.MouseOverBackColor = Color.FromArgb(15, 52, 66);
            liveButton.FlatAppearance.MouseDownBackColor = Color.FromArgb(7, 8, 9);
            reloadButton = MakeButton("重连");
            var help = MakeButton("说明");
            header.Controls.Add(liveButton, 2, 0);
            header.Controls.Add(reloadButton, 3, 0);
            connection.Dock = DockStyle.Fill;
            connection.TextAlign = ContentAlignment.MiddleRight;
            connection.ForeColor = Color.FromArgb(169, 178, 188);
            connection.AutoEllipsis = true;
            connection.Margin = new Padding(4, 0, 4, 0);
            connection.Text = "准备启动";
            header.Controls.Add(connection, 5, 0);
            header.Controls.Add(help, 6, 0);

            var content = new Panel { Dock = DockStyle.Fill };
            view.Dock = DockStyle.Fill;
            view.DefaultBackgroundColor = BackColor;
            cover.Dock = DockStyle.Fill;
            cover.BackColor = BackColor;
            var notice = new TableLayoutPanel { Dock = DockStyle.Fill, Padding = new Padding(64), ColumnCount = 1, RowCount = 5 };
            notice.RowStyles.Add(new RowStyle(SizeType.Percent, 50));
            notice.RowStyles.Add(new RowStyle(SizeType.Absolute, 64));
            notice.RowStyles.Add(new RowStyle(SizeType.Absolute, 150));
            notice.RowStyles.Add(new RowStyle(SizeType.Absolute, 48));
            notice.RowStyles.Add(new RowStyle(SizeType.Percent, 50));
            coverTitle.Font = new Font("Microsoft YaHei UI", 22F, FontStyle.Bold);
            coverTitle.ForeColor = Color.FromArgb(230, 233, 237);
            coverTitle.Dock = DockStyle.Fill;
            coverTitle.TextAlign = ContentAlignment.MiddleCenter;
            coverBody.ForeColor = Color.FromArgb(169, 178, 188);
            coverBody.Dock = DockStyle.Fill;
            coverBody.TextAlign = ContentAlignment.MiddleCenter;
            coverBody.Font = new Font("Microsoft YaHei UI", 11.5F);
            notice.Controls.Add(coverTitle, 0, 1);
            notice.Controls.Add(coverBody, 0, 2);
            cover.Controls.Add(notice);
            content.Controls.Add(view);
            content.Controls.Add(cover);
            Controls.Add(content);
            Controls.Add(new Panel { Dock = DockStyle.Top, Height = 1, BackColor = Color.FromArgb(49, 56, 65) });
            Controls.Add(header);
            liveButton.Click += async (s, e) => await ConnectLive();
            reloadButton.Click += async (s, e) => await ConnectLive();
            address.KeyDown += async (s, e) => { if (e.KeyCode == Keys.Enter) { e.SuppressKeyPress = true; await ConnectLive(); } };
            address.TextChanged += (s, e) =>
            {
                if (updatingAddress || closing) return;
                // Editing always wins over a pending response from the old address.
                reconnectTimer.Stop();
                navigationGeneration++;
                if (connectionCancellation != null) connectionCancellation.Cancel();
                connection.Text = "地址已修改，回车连接";
            };
            reconnectTimer.Tick += async (s, e) =>
            {
                if (!closing && !connectionInFlight) await ConnectLive(true);
            };
            help.Click += (s, e) => MessageBox.Show(this,
                "连接 X5：输入板子的局域网 IPv4 地址，端口为 8090。连接成功并且任务节点就绪后可直接下发真实任务，无需解锁或输入控制令牌。\n\n界面中的图像、识别结果、任务进度来自小车。服务未连接时不生成目标、识别结果或成功状态。\n\n左侧栏目只切换显示页面。下发任务后由 X5 节点与 FPGA 执行，以板端反馈确认进度与完成。\n\n语音来自车载 USB 麦克风。关闭本软件不会停止正在执行的板端任务；需要停止时请使用对应停止按钮并查看反馈。\n\n实时算法和硬件控制在 X5 上执行，电脑通过局域网显示和下发指令。",
                "消防救援控制台 · 使用说明", MessageBoxButtons.OK, MessageBoxIcon.Information);
            FormClosed += (s, e) =>
            {
                closing = true;
                navigationGeneration++;
                reconnectTimer.Stop();
                reconnectTimer.Dispose();
                if (connectionCancellation != null) connectionCancellation.Cancel();
                http.Dispose();
                view.Dispose();
            };
            Shown += async (s, e) =>
            {
                // Set this after the initial DPI/layout pass, which can update
                // the form bounds while its native window is being created.
                WindowState = FormWindowState.Maximized;
                await ConnectLive();
            };
            AutoScaleDimensions = new SizeF(96F, 96F);
            AutoScaleMode = AutoScaleMode.Dpi;
            ResumeLayout(false);
            PerformLayout();
        }

        static Button MakeButton(string text)
        {
            var b = new Button { Text = text, Dock = DockStyle.Fill, FlatStyle = FlatStyle.Flat, Margin = new Padding(8, 0, 0, 0), BackColor = Color.FromArgb(34, 39, 46), ForeColor = Color.FromArgb(230, 233, 237), Cursor = Cursors.Hand, TabStop = true };
            b.FlatAppearance.BorderColor = Color.FromArgb(72, 81, 92);
            b.FlatAppearance.MouseOverBackColor = Color.FromArgb(43, 49, 58);
            b.FlatAppearance.MouseDownBackColor = Color.FromArgb(7, 8, 9);
            return b;
        }

        protected override void OnHandleCreated(EventArgs e)
        {
            base.OnHandleCreated(e);
            try
            {
                int dark = 1;
                DwmSetWindowAttribute(Handle, 20, ref dark, sizeof(int));
                int caption = 0x00100D0B;
                DwmSetWindowAttribute(Handle, 35, ref caption, sizeof(int));
                int text = 0x00EDE9E6;
                DwmSetWindowAttribute(Handle, 36, ref text, sizeof(int));
            }
            catch (DllNotFoundException) { }
            catch (EntryPointNotFoundException) { }
        }

        void SetBusy(bool busy)
        {
            liveButton.Enabled = reloadButton.Enabled = !busy;
            // Allow a manually corrected address to cancel an automatic attempt.
            address.Enabled = true;
        }

        void ScheduleReconnect(string detail)
        {
            if (closing) return;
            ShowCover("等待 X5，自动重连中", "请保持电脑与 X5 在同一局域网，等待板端服务启动。\n约每 3 秒只读检查一次；连接成功后自动显示，不重发任务。\n修改顶部地址后按回车可立即切换。\n\n" + detail);
            connection.Text = "等待 X5 · 自动重连";
            reconnectTimer.Start();
        }

        void ShowCover(string title, string body)
        {
            coverTitle.Text = title;
            coverBody.Text = body;
            cover.Visible = true;
            cover.BringToFront();
        }

        string ReadAddress()
        {
            try
            {
                var candidate = File.ReadAllText(Path.Combine(dataDirectory, "connection.txt"), Encoding.UTF8).Trim();
                Uri valid;
                if (TryLiveAddress(candidate, out valid)) return valid.Authority;
            }
            catch (IOException) { }
            catch (UnauthorizedAccessException) { }
            return "192.168.127.10:8090";
        }

        internal static bool TryLiveAddress(string text, out Uri uri)
        {
            uri = null;
            var value = (text ?? "").Trim();
            if (!value.StartsWith("http://", StringComparison.OrdinalIgnoreCase)) value = "http://" + value;
            Uri parsed;
            if (!Uri.TryCreate(value, UriKind.Absolute, out parsed) || parsed.Scheme != "http" || parsed.Port != 8090 || parsed.AbsolutePath != "/" || parsed.Query.Length != 0 || parsed.Fragment.Length != 0 || parsed.UserInfo.Length != 0) return false;
            var parts = parsed.Host.Split('.');
            if (parts.Length != 4) return false;
            var bytes = new byte[4];
            for (int i = 0; i < 4; i++) if (!Byte.TryParse(parts[i], out bytes[i]) || parts[i] != bytes[i].ToString()) return false;
            bool local = bytes[0] == 127 || bytes[0] == 10 || (bytes[0] == 172 && bytes[1] >= 16 && bytes[1] <= 31) || (bytes[0] == 192 && bytes[1] == 168) || (bytes[0] == 169 && bytes[1] == 254);
            if (!local) return false;
            // Reject parser normalization of alternative IPv4 spellings and escaped authorities.
            var originalAuthority = value.Substring(7).TrimEnd('/');
            if (!String.Equals(originalAuthority, parsed.Host + ":8090", StringComparison.Ordinal)) return false;
            uri = parsed;
            return true;
        }

        async Task<bool> EnsureBrowser()
        {
            if (ready) return true;
            if (initializing) return false;
            initializing = true;
            try
            {
                Directory.CreateDirectory(dataDirectory);
                var options = new CoreWebView2EnvironmentOptions("--disable-features=msEdgeSidebarV2");
                var env = await CoreWebView2Environment.CreateAsync(null, Path.Combine(dataDirectory, "WebView2"), options);
                if (closing) return false;
                await view.EnsureCoreWebView2Async(env);
                view.ZoomFactor = 1.0;
                var core = view.CoreWebView2;
                core.Settings.AreDefaultContextMenusEnabled = false;
                core.Settings.AreDevToolsEnabled = false;
                core.Settings.AreBrowserAcceleratorKeysEnabled = false;
                core.Settings.IsStatusBarEnabled = false;
                core.Settings.IsZoomControlEnabled = false;
                core.Settings.IsPinchZoomEnabled = false;
                core.Settings.IsGeneralAutofillEnabled = false;
                core.Settings.IsPasswordAutosaveEnabled = false;
                core.Settings.IsWebMessageEnabled = false;
                core.NewWindowRequested += (s, e) => e.Handled = true;
                core.PermissionRequested += (s, e) => e.State = CoreWebView2PermissionState.Deny;
                core.NavigationStarting += (s, e) =>
                {
                    if (e.Uri == "about:blank") return;
                    if (!SameOrigin(e.Uri)) { e.Cancel = true; connection.Text = "已阻止外部跳转"; }
                    else pageNavigationId = e.NavigationId;
                };
                core.NavigationCompleted += (s, e) =>
                {
                    if (closing || core.Source == "about:blank" || allowedOrigin == null || pageGeneration != navigationGeneration || e.NavigationId != pageNavigationId) return;
                    if (e.IsSuccess && SameOrigin(core.Source))
                    {
                        reconnectTimer.Stop();
                        cover.Visible = false;
                        connection.Text = "●  X5 · 真实控制";
                    }
                    else ScheduleReconnect("页面载入失败：" + e.WebErrorStatus);
                };
                core.ProcessFailed += (s, e) => { if (!closing) { ShowCover("显示进程已退出", "请关闭并重新打开控制台。板端任务不会因此停止。\n" + e.ProcessFailedKind); connection.Text = "显示进程异常"; } };
                core.AddWebResourceRequestedFilter("*", CoreWebView2WebResourceContext.All);
                core.WebResourceRequested += (s, e) =>
                {
                    string uri = e.Request.Uri;
                    if ((uri.StartsWith("http:", StringComparison.OrdinalIgnoreCase) || uri.StartsWith("https:", StringComparison.OrdinalIgnoreCase)) && !SameOrigin(uri))
                        e.Response = env.CreateWebResourceResponse(new MemoryStream(new byte[0]), 403, "External resource denied", "Content-Type: text/plain");
                };
                core.DownloadStarting += DownloadStarting;
                ready = true;
                return true;
            }
            catch (Exception ex)
            {
                if (!closing)
                {
                    ShowCover("桌面显示组件暂不可用", "需要 Microsoft Edge WebView2 Runtime。\n当前电脑通常随 Edge 自带；若被移除，请安装微软官方 Runtime 后重试。\n" + ex.Message);
                    connection.Text = "WebView2 初始化失败";
                }
                return false;
            }
            finally { initializing = false; }
        }

        bool SameOrigin(string value)
        {
            Uri candidate;
            return allowedOrigin != null && Uri.TryCreate(value, UriKind.Absolute, out candidate) && candidate.Scheme == allowedOrigin.Scheme && candidate.Host == allowedOrigin.Host && candidate.Port == allowedOrigin.Port && candidate.UserInfo.Length == 0;
        }

        void DownloadStarting(object sender, CoreWebView2DownloadStartingEventArgs e)
        {
            e.Handled = true;
            string uri = e.DownloadOperation.Uri;
            bool blobFromPage = allowedOrigin != null && uri.StartsWith("blob:" + allowedOrigin.GetLeftPart(UriPartial.Authority) + "/", StringComparison.OrdinalIgnoreCase);
            if (!SameOrigin(uri) && !blobFromPage) { e.Cancel = true; return; }
            var deferral = e.GetDeferral();
            BeginInvoke(new Action(() =>
            {
                try
                {
                    using (var dialog = new SaveFileDialog { Title = "保存小车状态快照", Filter = "JSON 状态文件 (*.json)|*.json", FileName = "rescue_snapshot_" + DateTime.Now.ToString("yyyyMMdd_HHmmss") + ".json", DefaultExt = "json", AddExtension = true, OverwritePrompt = true })
                    {
                        if (closing || dialog.ShowDialog(this) != DialogResult.OK) { e.Cancel = true; return; }
                        e.ResultFilePath = dialog.FileName;
                    }
                }
                finally { deferral.Complete(); }
            }));
        }

        async Task ConnectLive(bool automatic = false)
        {
            if (closing) return;
            reconnectTimer.Stop();
            Uri target;
            if (!TryLiveAddress(address.Text, out target))
            {
                if (!automatic) MessageBox.Show(this, "请输入 X5 的局域网 IPv4 地址和端口，例如 192.168.127.10:8090。\n仅支持私网或本机地址，实时服务端口固定为 8090。", "检查连接地址", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            if (connectionInFlight)
            {
                navigationGeneration++;
                if (connectionCancellation != null) connectionCancellation.Cancel();
                // Wait for the cancelled attempt to finish; never overlap requests.
                reconnectTimer.Start();
                return;
            }
            connectionInFlight = true;
            var cancellation = new CancellationTokenSource();
            connectionCancellation = cancellation;
            SetBusy(true);
            var generation = ++navigationGeneration;
            try
            {
                allowedOrigin = null;
                if (ready) view.CoreWebView2.Navigate("about:blank");
                ShowCover("正在连接 X5", target.Authority + "\n只读取服务状态，不发送任务指令。");
                connection.Text = automatic ? "自动重连 · 检查服务…" : "检查 X5 服务…";
                Text = "消防救援控制台 · 连接 X5";
                var state = await GetState(new Uri(target, "/api/state"), cancellation.Token);
                if (closing || generation != navigationGeneration) return;
                if (!IsLiveState(state))
                {
                    ShowCover("已拒绝连接", "该地址未提供真实小车服务标识。请使用 X5 上的实时服务。");
                    connection.Text = "服务身份不符";
                    return;
                }
                if (!await EnsureBrowser()) return;
                if (closing || generation != navigationGeneration) return;
                allowedOrigin = target;
                updatingAddress = true;
                try { address.Text = target.Authority; }
                finally { updatingAddress = false; }
                try { File.WriteAllText(Path.Combine(dataDirectory, "connection.txt"), target.Authority, Encoding.UTF8); } catch (IOException) { } catch (UnauthorizedAccessException) { }
                Text = "消防救援控制台 · X5 " + target.Authority;
                pageGeneration = generation;
                view.CoreWebView2.Navigate(target.AbsoluteUri);
            }
            catch (Exception ex)
            {
                if (!closing && generation == navigationGeneration) ScheduleReconnect(SafeError(ex));
            }
            finally
            {
                connectionCancellation = null;
                cancellation.Dispose();
                connectionInFlight = false;
                if (!closing) SetBusy(false);
            }
        }

        internal static bool IsLiveState(Dictionary<string, object> state)
        {
            object synthetic, mode;
            return state != null && state.TryGetValue("synthetic", out synthetic) && synthetic is bool && !(bool)synthetic
                && state.TryGetValue("mode", out mode) && mode is string && (string)mode == "live";
        }

        async Task<Dictionary<string, object>> GetState(Uri uri, CancellationToken cancellationToken)
        {
            using (var deadline = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken))
            {
                deadline.CancelAfter(TimeSpan.FromSeconds(5));
                using (var response = await http.GetAsync(uri, HttpCompletionOption.ResponseHeadersRead, deadline.Token))
                {
                    response.EnsureSuccessStatusCode();
                    if (response.Content.Headers.ContentLength > 1024 * 1024) throw new InvalidDataException("状态响应过大。");
                    using (var stream = await response.Content.ReadAsStreamAsync())
                    using (var buffer = new MemoryStream())
                    {
                        var chunk = new byte[8192]; int n;
                        while ((n = await stream.ReadAsync(chunk, 0, chunk.Length, deadline.Token)) > 0)
                        {
                            if (buffer.Length + n > 1024 * 1024) throw new InvalidDataException("状态响应过大。");
                            buffer.Write(chunk, 0, n);
                        }
                        return new JavaScriptSerializer { MaxJsonLength = 1024 * 1024 }.Deserialize<Dictionary<string, object>>(Encoding.UTF8.GetString(buffer.ToArray()));
                    }
                }
            }
        }

        static string SafeError(Exception ex) { return ex is TaskCanceledException ? "服务响应超时。" : ex.Message; }
    }
}
