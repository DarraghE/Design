import 'dart:async';

import 'package:flutter/services.dart';
import 'package:material_ui/material_ui.dart';

import '../app.dart';
import '../closepass/incident.dart';
import '../link/main_module.dart';
import '../link/protocol.dart';
import '../services/location.dart';
import 'connect_sheet.dart';
import 'incident_screen.dart';
import 'navigation_screen.dart';
import 'recordings_screen.dart';
import 'settings_screen.dart';
import 'theme.dart';
import 'widgets/lcd_preview.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  /// Off in widget tests (there is no location plugin there).
  static bool askPermissions = true;

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  StreamSubscription<ClosePassAlert>? _alerts;
  StreamSubscription<Incident>? _incidents;

  @override
  void initState() {
    super.initState();
    if (HomeScreen.askPermissions) WidgetsBinding.instance.addPostFrameCallback((_) => askForLocation());
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    final app = AppScope.of(context);
    _alerts ??= app.closePass.alertEvents.listen(_onAlert);
    _incidents ??= app.closePass.incidentEvents.listen(_onIncident);
  }

  @override
  void dispose() {
    _alerts?.cancel();
    _incidents?.cancel();
    super.dispose();
  }

  void _onAlert(ClosePassAlert a) {
    HapticFeedback.heavyImpact();
    final parts = [
      if (a.minDistanceM != null) '${a.minDistanceM!.toStringAsFixed(1)} m',
      if (a.speedKmh != null) '${a.speedKmh!.round()} km/h',
    ];
    ScaffoldMessenger.of(context)
      ..hideCurrentSnackBar()
      ..showSnackBar(SnackBar(
        backgroundColor: Palette.danger,
        content: Text('Close pass${parts.isEmpty ? '' : ' · ${parts.join(' · ')}'} — receiving photos and LiDAR…',
            style: const TextStyle(color: Colors.white, fontWeight: FontWeight.w600)),
      ));
  }

  void _onIncident(Incident i) {
    ScaffoldMessenger.of(context)
      ..hideCurrentSnackBar()
      ..showSnackBar(SnackBar(
        content: const Text('Close pass recorded. Reading the number plate…'),
        action: SnackBarAction(
          label: 'View',
          onPressed: () => Navigator.of(context).push(MaterialPageRoute<void>(builder: (_) => IncidentScreen(incidentId: i.id))),
        ),
      ));
  }

  @override
  Widget build(BuildContext context) {
    final app = AppScope.of(context);
    return Scaffold(
      body: SafeArea(
        child: ListenableBuilder(
          listenable: Listenable.merge([app.module, app.display, app.nav, app.store, app.closePass]),
          builder: (context, _) => ListView(
            padding: const EdgeInsets.fromLTRB(20, 12, 20, 28),
            children: [
              Row(children: [
                const Text('Bike Nav', style: TextStyle(fontSize: 30, fontWeight: FontWeight.w800, letterSpacing: -0.5)),
                const Spacer(),
                IconButton(
                  tooltip: 'Settings',
                  icon: const Icon(Icons.settings_outlined),
                  onPressed: () => Navigator.of(context).push(MaterialPageRoute<void>(builder: (_) => const SettingsScreen())),
                ),
              ]),
              const SizedBox(height: 10),
              _ConnectionCard(app: app),
              const SizedBox(height: 22),
              const SectionLabel('Handlebar screen'),
              LcdPreview(frame: app.display.frame, flash: app.display.flash),
              Padding(
                padding: const EdgeInsets.only(top: 8, left: 4),
                child: Text(
                  app.module.connected
                      ? 'Live: this is what the handlebar shows.'
                      : 'Preview only: connect the Main Module to send it.',
                  style: const TextStyle(color: Palette.muted, fontSize: 13),
                ),
              ),
              const SizedBox(height: 22),
              const SectionLabel('Modes'),
              _NavigationCard(app: app),
              const SizedBox(height: 12),
              _ClosePassCard(app: app),
            ],
          ),
        ),
      ),
    );
  }
}

class _ConnectionCard extends StatelessWidget {
  final AppServices app;
  const _ConnectionCard({required this.app});

  @override
  Widget build(BuildContext context) {
    final m = app.module;
    final (colour, title, subtitle) = switch (m.state) {
      LinkState.connected => (
          Palette.ok,
          m.isDemo ? 'Demo module connected' : '${m.name ?? 'Main Module'} connected',
          _statusLine(m),
        ),
      LinkState.connecting => (Palette.warning, 'Connecting to ${m.name ?? 'Main Module'}…', m.error ?? 'Make sure it is switched on and nearby'),
      LinkState.disconnected => (Palette.muted, 'Main Module not connected', 'Tap to connect'),
    };
    return Card(
      child: InkWell(
        borderRadius: BorderRadius.circular(22),
        onTap: () => showConnectSheet(context),
        child: Padding(
          padding: const EdgeInsets.all(16),
          child: Row(children: [
            Container(
              width: 12,
              height: 12,
              decoration: BoxDecoration(
                color: colour,
                shape: BoxShape.circle,
                boxShadow: [BoxShadow(color: colour.withValues(alpha: 0.6), blurRadius: 10)],
              ),
            ),
            const SizedBox(width: 14),
            Expanded(
              child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                Text(title, style: const TextStyle(fontSize: 16, fontWeight: FontWeight.w700)),
                const SizedBox(height: 2),
                Text(subtitle, maxLines: 2, overflow: TextOverflow.ellipsis, style: const TextStyle(color: Palette.muted, fontSize: 13)),
              ]),
            ),
            if (m.transferProgress != null)
              SizedBox(width: 26, height: 26, child: CircularProgressIndicator(value: m.transferProgress, strokeWidth: 3))
            else
              const Icon(Icons.bluetooth, color: Palette.muted),
          ]),
        ),
      ),
    );
  }

  static String _statusLine(MainModule m) {
    final s = m.status;
    if (m.transferProgress != null) return 'Receiving close-pass data… ${(m.transferProgress! * 100).round()}%';
    if (s == null) return 'Close Pass armed';
    final down = [
      if (!s.lidarOk) 'LiDAR',
      if (!s.cameraOk) 'camera',
      if (!s.frontOk) 'screen',
    ];
    final parts = [
      down.isEmpty ? 'LiDAR, camera and screen OK' : 'Check: ${down.join(', ')}',
      if (s.batteryPercent != null) 'battery ${s.batteryPercent}%',
    ];
    return parts.join('  ·  ');
  }
}

class _ModeCard extends StatelessWidget {
  final IconData icon;
  final Color colour;
  final String title;
  final String status;
  final bool on;
  final String detail;
  final VoidCallback onTap;
  final Widget? action;
  const _ModeCard({
    required this.icon,
    required this.colour,
    required this.title,
    required this.status,
    required this.on,
    required this.detail,
    required this.onTap,
    this.action,
  });

  @override
  Widget build(BuildContext context) {
    return Card(
      child: InkWell(
        borderRadius: BorderRadius.circular(22),
        onTap: onTap,
        child: Padding(
          padding: const EdgeInsets.all(18),
          child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
            Row(children: [
              Container(
                padding: const EdgeInsets.all(10),
                decoration: BoxDecoration(color: colour.withValues(alpha: 0.14), borderRadius: BorderRadius.circular(14)),
                child: Icon(icon, color: colour),
              ),
              const SizedBox(width: 14),
              Expanded(child: Text(title, style: const TextStyle(fontSize: 19, fontWeight: FontWeight.w800))),
              Container(
                padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
                decoration: BoxDecoration(
                  color: on ? colour.withValues(alpha: 0.16) : Palette.surfaceHigh,
                  borderRadius: BorderRadius.circular(20),
                ),
                child: Text(status, style: TextStyle(color: on ? colour : Palette.muted, fontWeight: FontWeight.w700, fontSize: 12)),
              ),
            ]),
            const SizedBox(height: 12),
            Text(detail, style: const TextStyle(color: Palette.muted, fontSize: 14, height: 1.35)),
            if (action != null) ...[const SizedBox(height: 14), action!],
          ]),
        ),
      ),
    );
  }
}

class _NavigationCard extends StatelessWidget {
  final AppServices app;
  const _NavigationCard({required this.app});

  @override
  Widget build(BuildContext context) {
    final nav = app.nav;
    final s = nav.state;
    final detail = !nav.active
        ? 'Turn-by-turn arrows on the handlebar screen. Choose a destination to start.'
        : s == null
            ? 'Getting the route…'
            : s.arrived
                ? 'You have arrived at ${nav.destination?.label ?? 'your destination'}.'
                : (s.next?.instruction.isNotEmpty ?? false)
                    ? s.next!.instruction
                    : 'Navigating to ${nav.destination?.label ?? 'destination'}';
    return _ModeCard(
      icon: Icons.navigation_rounded,
      colour: Palette.accent,
      title: 'Navigation',
      status: nav.active ? (nav.simulated ? 'SIMULATED' : 'ON') : 'OFF',
      on: nav.active,
      detail: detail,
      onTap: () => Navigator.of(context).push(MaterialPageRoute<void>(builder: (_) => const NavigationScreen())),
      action: nav.active
          ? Row(children: [
              Expanded(
                child: FilledButton.tonal(
                  onPressed: () => Navigator.of(context).push(MaterialPageRoute<void>(builder: (_) => const NavigationScreen())),
                  child: const Text('Open'),
                ),
              ),
              const SizedBox(width: 10),
              Expanded(child: OutlinedButton(onPressed: () => nav.stop(), child: const Text('End'))),
            ])
          : null,
    );
  }
}

class _ClosePassCard extends StatelessWidget {
  final AppServices app;
  const _ClosePassCard({required this.app});

  @override
  Widget build(BuildContext context) {
    final count = app.store.incidents.length;
    final last = app.store.incidents.isEmpty ? null : app.store.incidents.first;
    final armed = app.module.connected;
    final lines = [
      armed
          ? (app.nav.active
              ? 'Armed. While navigating, close passes are recorded but the handlebar keeps showing directions.'
              : 'Armed. A close pass shows on the handlebar, and the photos and LiDAR are saved here.')
          : 'Connect the Main Module to arm it.',
      if (last != null) 'Last: ${_ago(last.time)}${last.plate?.found == true ? ' · ${last.plate!.plate}' : ''}',
    ];
    return _ModeCard(
      icon: Icons.shield_outlined,
      colour: Palette.danger,
      title: 'Close Pass',
      status: armed ? 'ARMED' : 'WAITING',
      on: armed,
      detail: lines.join('\n'),
      onTap: () => Navigator.of(context).push(MaterialPageRoute<void>(builder: (_) => const RecordingsScreen())),
      action: Row(children: [
        Expanded(
          child: FilledButton.tonal(
            onPressed: () => Navigator.of(context).push(MaterialPageRoute<void>(builder: (_) => const RecordingsScreen())),
            child: Text(count == 0 ? 'Recordings' : 'Recordings ($count)'),
          ),
        ),
        if (app.demo != null) ...[
          const SizedBox(width: 10),
          Expanded(
            child: OutlinedButton.icon(
              onPressed: () => app.demo!.triggerClosePass(),
              icon: const Icon(Icons.bolt, size: 18),
              label: const Text('Simulate pass'),
            ),
          ),
        ],
      ]),
    );
  }
}

String _ago(DateTime t) {
  final d = DateTime.now().difference(t);
  if (d.inMinutes < 1) return 'just now';
  if (d.inHours < 1) return '${d.inMinutes} min ago';
  if (d.inDays < 1) return '${d.inHours} h ago';
  return '${t.day}/${t.month} ${t.hour.toString().padLeft(2, '0')}:${t.minute.toString().padLeft(2, '0')}';
}
