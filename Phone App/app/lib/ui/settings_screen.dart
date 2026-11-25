import 'package:material_ui/material_ui.dart';

import '../app.dart';
import '../closepass/incident.dart';
import 'package:bike_navigation/bike_navigation.dart' show OrsClient;

import '../closepass/plate_reader.dart';
import '../closepass/uploader.dart';
import '../link/protocol.dart';
import '../settings.dart';
import 'connect_sheet.dart';
import 'theme.dart';

/// The few things to set up once: routing key, OpenAI key, website.
class SettingsScreen extends StatefulWidget {
  const SettingsScreen({super.key});

  @override
  State<SettingsScreen> createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  late final TextEditingController _ors, _openAi, _model, _url, _token;
  bool _init = false;
  String? _orsCheck, _openAiCheck, _uploadCheck;
  bool _checking = false;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    if (_init) return;
    _init = true;
    final s = AppScope.of(context).settings;
    _ors = TextEditingController(text: s.orsKey);
    _openAi = TextEditingController(text: s.openAiKey);
    _model = TextEditingController(text: s.openAiModel);
    _url = TextEditingController(text: s.uploadUrl);
    _token = TextEditingController(text: s.uploadToken);
  }

  @override
  void dispose() {
    for (final c in [_ors, _openAi, _model, _url, _token]) {
      c.dispose();
    }
    super.dispose();
  }

  Future<void> _save() async {
    final app = AppScope.of(context);
    await app.settings.update(
      orsKey: _ors.text,
      openAiKey: _openAi.text,
      openAiModel: _model.text,
      uploadUrl: _url.text,
      uploadToken: _token.text,
    );
    // New keys or a new website: finish anything that was waiting for them.
    await app.closePass.resumePending();
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Saved')));
    }
  }

  Future<void> _checkOrs() async {
    await _save();
    if (!mounted) return;
    setState(() {
      _checking = true;
      _orsCheck = null;
    });
    final result = await OrsClient(apiKey: AppScope.of(context).settings.orsKey).check();
    if (mounted) {
      setState(() {
        _checking = false;
        _orsCheck = result ?? 'Key works. Routes and place search are ready.';
      });
    }
  }

  Future<void> _checkOpenAi() async {
    await _save();
    if (!mounted) return;
    setState(() {
      _checking = true;
      _openAiCheck = null;
    });
    final s = AppScope.of(context).settings;
    final result = await PlateReader(apiKey: s.openAiKey, model: s.openAiModel).check();
    if (mounted) {
      setState(() {
        _checking = false;
        _openAiCheck = result ?? 'Key works and ${s.openAiModel} is available.';
      });
    }
  }

  Future<void> _testUpload() async {
    await _save();
    if (!mounted) return;
    final s = AppScope.of(context).settings;
    final uri = s.uploadUri;
    if (uri == null) {
      setState(() => _uploadCheck = 'Enter an address starting with https://');
      return;
    }
    setState(() {
      _checking = true;
      _uploadCheck = null;
    });
    try {
      final test = Incident(
        id: 'test-${DateTime.now().millisecondsSinceEpoch}',
        time: DateTime.now(),
        distanceM: 0.85,
        speedKmh: 46,
        simulated: true,
        plate: const PlateResult(found: true, plate: '262-D-12345', country: 'IE', confidence: 0.9, plausibleFormat: true, notes: 'Test upload from the app settings'),
      );
      final r = await IncidentUploader(endpoint: uri, token: s.uploadToken).upload(test, photos: const []);
      setState(() => _uploadCheck = 'Website accepted the test upload${r.url == null ? '' : ': ${r.url}'}');
    } catch (e) {
      setState(() => _uploadCheck = e.toString());
    } finally {
      if (mounted) setState(() => _checking = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    final app = AppScope.of(context);
    return Scaffold(
      appBar: AppBar(
        title: const Text('Settings', style: TextStyle(fontWeight: FontWeight.w800)),
        actions: [TextButton(onPressed: _save, child: const Text('Save'))],
      ),
      body: ListView(
        padding: const EdgeInsets.fromLTRB(18, 4, 18, 40),
        children: [
          const SectionLabel('Main Module'),
          ListenableBuilder(
            listenable: Listenable.merge([app.module, app.settings]),
            builder: (context, _) => Card(
              child: ListTile(
                leading: const Icon(Icons.bluetooth),
                title: Text(app.module.connected ? '${app.module.name} connected' : (app.settings.deviceName?.isNotEmpty == true ? app.settings.deviceName! : 'No module chosen')),
                subtitle: const Text('Tap to connect, change or forget'),
                onTap: () => showConnectSheet(context),
              ),
            ),
          ),
          const SizedBox(height: 22),
          const SectionLabel('Navigation'),
          _field(_ors, 'OpenRouteService API key', secret: true,
              help: 'Free: sign up at account.heigit.org, then copy the key from its dashboard. Used for cycling routes and place search.'),
          const SizedBox(height: 8),
          OutlinedButton(onPressed: _checking ? null : _checkOrs, child: const Text('Check routing key')),
          if (_orsCheck != null) Padding(padding: const EdgeInsets.only(top: 8), child: Text(_orsCheck!, style: const TextStyle(color: Palette.muted))),
          const SizedBox(height: 22),
          const SectionLabel('Number plates (OpenAI)'),
          _field(_openAi, 'OpenAI API key', secret: true, help: 'Your own key from platform.openai.com. Photos are sent to OpenAI only to read the plate.'),
          const SizedBox(height: 10),
          _field(_model, 'Model', help: 'A vision model, e.g. ${Settings.defaultModel}, gpt-4.1-mini or gpt-4o.'),
          const SizedBox(height: 8),
          OutlinedButton(onPressed: _checking ? null : _checkOpenAi, child: const Text('Check OpenAI key')),
          if (_openAiCheck != null) Padding(padding: const EdgeInsets.only(top: 8), child: Text(_openAiCheck!, style: const TextStyle(color: Palette.muted))),
          const SizedBox(height: 22),
          const SectionLabel('Website'),
          _field(_url, 'Upload address', help: 'e.g. https://example.com/api/close-passes. Close passes with a confidently read plate are sent here automatically.'),
          const SizedBox(height: 10),
          _field(_token, 'Access token (optional)', secret: true, help: 'Sent as "Authorization: Bearer <token>".'),
          const SizedBox(height: 8),
          OutlinedButton(onPressed: _checking ? null : _testUpload, child: const Text('Send a test upload')),
          if (_uploadCheck != null) Padding(padding: const EdgeInsets.only(top: 8), child: Text(_uploadCheck!, style: const TextStyle(color: Palette.muted))),
          const SizedBox(height: 26),
          FilledButton(onPressed: _save, child: const Text('Save')),
          const SizedBox(height: 26),
          Center(
            child: Text('Bike Nav ${IncidentUploader.appVersion} · protocol v$protocolVersion',
                style: const TextStyle(color: Palette.muted, fontSize: 12)),
          ),
          TextButton(
            onPressed: () => showLicensePage(context: context, applicationName: 'Bike Nav', applicationVersion: IncidentUploader.appVersion),
            child: const Text('Open-source licences'),
          ),
        ],
      ),
    );
  }

  Widget _field(TextEditingController c, String label, {bool secret = false, String? help}) => TextField(
        controller: c,
        obscureText: secret,
        autocorrect: false,
        enableSuggestions: false,
        decoration: InputDecoration(labelText: label, helperText: help, helperMaxLines: 3),
      );
}
