import 'dart:async';

import 'package:material_ui/material_ui.dart';

import '../app.dart';
import '../link/main_module.dart';
import '../link/transport.dart';
import 'theme.dart';

Future<void> showConnectSheet(BuildContext context) => showModalBottomSheet<void>(
      context: context,
      isScrollControlled: true,
      showDragHandle: true,
      backgroundColor: Palette.surface,
      builder: (_) => const ConnectSheet(),
    );

/// Finds the Main Module and connects with one tap. The choice is
/// remembered, and the app reconnects by itself next time.
class ConnectSheet extends StatefulWidget {
  const ConnectSheet({super.key});

  @override
  State<ConnectSheet> createState() => _ConnectSheetState();
}

class _ConnectSheetState extends State<ConnectSheet> {
  List<FoundModule> _found = const [];
  StreamSubscription<List<FoundModule>>? _scan;
  bool _scanning = false;
  String? _error;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    final app = AppScope.of(context);
    if (_scan == null && !app.module.connected && app.finder != null) _startScan(app);
  }

  Future<void> _startScan(AppServices app) async {
    final finder = app.finder!;
    setState(() {
      _scanning = true;
      _error = null;
      _found = const [];
    });
    if (!await finder.ensureOn()) {
      if (!mounted) return;
      setState(() {
        _scanning = false;
        _error = 'Turn on Bluetooth to find the Main Module.';
      });
      return;
    }
    await _scan?.cancel();
    _scan = finder.scan().listen(
      (list) => mounted ? setState(() => _found = list) : null,
      onError: (Object e) => mounted ? setState(() => _error = 'Scanning failed: $e') : null,
      onDone: () => mounted ? setState(() => _scanning = false) : null,
    );
  }

  @override
  void dispose() {
    _scan?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final app = AppScope.of(context);
    return ListenableBuilder(
      listenable: app.module,
      builder: (context, _) {
        final m = app.module;
        return SafeArea(
          child: Padding(
            padding: const EdgeInsets.fromLTRB(20, 0, 20, 20),
            child: Column(mainAxisSize: MainAxisSize.min, crossAxisAlignment: CrossAxisAlignment.stretch, children: [
              const Text('Main Module', style: TextStyle(fontSize: 22, fontWeight: FontWeight.w800)),
              const SizedBox(height: 6),
              const Text('Switch the bike on, keep the phone close, then tap the module once. It will reconnect by itself from now on.',
                  style: TextStyle(color: Palette.muted, height: 1.35)),
              const SizedBox(height: 18),
              if (m.state != LinkState.disconnected) ...[
                Card(
                  color: Palette.surfaceHigh,
                  child: ListTile(
                    leading: Icon(m.connected ? Icons.check_circle : Icons.sync, color: m.connected ? Palette.ok : Palette.warning),
                    title: Text(m.connected ? '${m.name} connected' : 'Connecting to ${m.name ?? 'Main Module'}…'),
                    subtitle: m.error != null && !m.connected ? Text(m.error!, maxLines: 2, overflow: TextOverflow.ellipsis) : null,
                  ),
                ),
                const SizedBox(height: 12),
                Row(children: [
                  Expanded(
                    child: OutlinedButton(
                      onPressed: () async {
                        await app.disconnect();
                        if (context.mounted) _startScan(app);
                      },
                      child: const Text('Disconnect'),
                    ),
                  ),
                  if (app.settings.deviceId != null) ...[
                    const SizedBox(width: 10),
                    Expanded(
                      child: OutlinedButton(
                        onPressed: () async {
                          await app.disconnect();
                          await app.settings.rememberDevice(null, null);
                          if (context.mounted) _startScan(app);
                        },
                        child: const Text('Forget'),
                      ),
                    ),
                  ],
                ]),
              ] else ...[
                Row(children: [
                  const SectionLabel('Nearby'),
                  const Spacer(),
                  if (_scanning)
                    const Padding(
                      padding: EdgeInsets.only(bottom: 8, right: 4),
                      child: SizedBox(width: 16, height: 16, child: CircularProgressIndicator(strokeWidth: 2)),
                    )
                  else if (app.finder != null)
                    TextButton(onPressed: () => _startScan(app), child: const Text('Scan again')),
                ]),
                if (_error != null)
                  Padding(padding: const EdgeInsets.only(bottom: 8), child: Text(_error!, style: const TextStyle(color: Palette.danger))),
                if (_found.isEmpty)
                  Padding(
                    padding: const EdgeInsets.symmetric(vertical: 18),
                    child: Text(
                      _scanning ? 'Looking for "BikeNav Main"…' : 'No Main Module found.',
                      textAlign: TextAlign.center,
                      style: const TextStyle(color: Palette.muted),
                    ),
                  ),
                for (final f in _found)
                  Card(
                    color: Palette.surfaceHigh,
                    margin: const EdgeInsets.only(bottom: 8),
                    child: ListTile(
                      leading: const Icon(Icons.pedal_bike, color: Palette.accent),
                      title: Text(f.name),
                      subtitle: Text('Signal ${_bars(f.rssi)}'),
                      trailing: const Icon(Icons.chevron_right),
                      onTap: () async {
                        await _scan?.cancel();
                        await app.finder?.stopScan();
                        await app.module.connectTo(f);
                      },
                    ),
                  ),
              ],
              const SizedBox(height: 14),
              const Divider(),
              TextButton.icon(
                onPressed: () async {
                  await _scan?.cancel();
                  await app.connectDemo();
                  if (context.mounted) Navigator.of(context).pop();
                },
                icon: const Icon(Icons.science_outlined),
                label: const Text('No hardware? Use the demo module'),
              ),
            ]),
          ),
        );
      },
    );
  }

  static String _bars(int rssi) => rssi > -60 ? 'strong' : (rssi > -75 ? 'good' : 'weak');
}
