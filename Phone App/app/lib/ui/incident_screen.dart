
import 'package:flutter/services.dart';
import 'package:material_ui/material_ui.dart';

import '../app.dart';
import '../closepass/clip_analysis.dart';
import '../closepass/incident.dart';
import '../link/close_pass_package.dart';
import 'recordings_screen.dart';
import 'theme.dart';
import 'widgets/lidar_replay.dart';

/// One close pass: the LiDAR reconstruction, the photos, the plate and the
/// upload.
class IncidentScreen extends StatefulWidget {
  final String incidentId;
  const IncidentScreen({super.key, required this.incidentId});

  @override
  State<IncidentScreen> createState() => _IncidentScreenState();
}

class _IncidentScreenState extends State<IncidentScreen> {
  LidarClip? _clip;
  ClipAnalysis? _analysis;
  List<Uint8List> _photos = const [];
  bool _loaded = false;
  bool _busy = false;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    if (!_loaded) {
      _loaded = true;
      _load();
    }
  }

  Future<void> _load() async {
    final app = AppScope.of(context);
    final i = app.store.byId(widget.incidentId);
    if (i == null) return;
    final clip = await app.store.clip(i);
    final photos = await app.store.photos(i);
    if (!mounted) return;
    setState(() {
      _clip = clip;
      _analysis = clip == null ? null : ClipAnalysis.of(clip);
      _photos = photos;
    });
  }

  Future<void> _run(Future<void> Function() job) async {
    setState(() => _busy = true);
    try {
      await job();
    } finally {
      if (mounted) setState(() => _busy = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    final app = AppScope.of(context);
    return ListenableBuilder(
      listenable: app.store,
      builder: (context, _) {
        final i = app.store.byId(widget.incidentId);
        if (i == null) return const Scaffold(body: Center(child: Text('This recording was deleted.')));
        return Scaffold(
          appBar: AppBar(
            title: Text(formatWhen(i.time), style: const TextStyle(fontSize: 16, fontWeight: FontWeight.w700)),
            actions: [
              IconButton(
                tooltip: 'Delete',
                icon: const Icon(Icons.delete_outline),
                onPressed: () async {
                  final ok = await showDialog<bool>(
                    context: context,
                    builder: (c) => AlertDialog(
                      title: const Text('Delete this recording?'),
                      content: const Text('Its photos and LiDAR data are removed from this phone. Anything already uploaded stays on the website.'),
                      actions: [
                        TextButton(onPressed: () => Navigator.pop(c, false), child: const Text('Cancel')),
                        TextButton(onPressed: () => Navigator.pop(c, true), child: const Text('Delete')),
                      ],
                    ),
                  );
                  if (ok == true) {
                    await app.store.delete(i);
                    if (context.mounted) Navigator.of(context).pop();
                  }
                },
              ),
            ],
          ),
          body: ListView(
            padding: const EdgeInsets.fromLTRB(16, 4, 16, 28),
            children: [
              if (i.simulated)
                const Padding(
                  padding: EdgeInsets.only(bottom: 10),
                  child: Text('Simulated close pass (demo module)', style: TextStyle(color: Palette.warning, fontWeight: FontWeight.w600)),
                ),
              _stats(i),
              const SizedBox(height: 18),
              const SectionLabel('LiDAR reconstruction'),
              if (_clip != null && _analysis != null)
                LidarReplay(clip: _clip!, analysis: _analysis!)
              else
                const AspectRatio(aspectRatio: 1, child: Center(child: CircularProgressIndicator())),
              const SizedBox(height: 18),
              const SectionLabel('Number plate'),
              _plateCard(app, i),
              const SizedBox(height: 18),
              SectionLabel('Photos (${i.photoFiles.length})'),
              _photoStrip(i),
              const SizedBox(height: 18),
              const SectionLabel('Website'),
              _uploadCard(app, i),
            ],
          ),
        );
      },
    );
  }

  Widget _stats(Incident i) {
    Widget stat(String value, String label, Color colour) => Expanded(
          child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
            Text(value, style: TextStyle(fontSize: 28, fontWeight: FontWeight.w800, color: colour)),
            Text(label, style: const TextStyle(color: Palette.muted, fontSize: 12)),
          ]),
        );
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(18),
        child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
          Row(children: [
            stat(i.distanceM == null ? '–' : '${i.distanceM!.toStringAsFixed(2)} m', 'closest distance', Palette.danger),
            stat(i.speedKmh == null ? '–' : '${i.speedKmh!.round()} km/h', 'vehicle speed', Palette.warning),
          ]),
          const SizedBox(height: 12),
          Row(children: [
            const Icon(Icons.place_outlined, size: 16, color: Palette.muted),
            const SizedBox(width: 6),
            Expanded(
              child: Text(
                i.hasLocation
                    ? '${i.lat!.toStringAsFixed(5)}, ${i.lng!.toStringAsFixed(5)}  (±${(i.accuracyM ?? 0).round()} m)'
                    : 'No location (location was off)',
                style: const TextStyle(color: Palette.muted, fontSize: 13),
              ),
            ),
          ]),
        ]),
      ),
    );
  }

  Widget _plateCard(AppServices app, Incident i) {
    final p = i.plate;
    final reading = i.plateStatus == PlateStatus.reading || (_busy && i.plateStatus == PlateStatus.waiting);
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(18),
        child: Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: [
          if (p != null && p.found) ...[
            Center(child: _PlateBadge(plate: p.plate, irish: p.country == 'IE')),
            const SizedBox(height: 12),
            Text(
              [
                '${(p.confidence * 100).round()}% confident',
                if (!p.plausibleFormat) 'unusual plate format',
                if (p.vehicleSummary.isNotEmpty) p.vehicleSummary,
              ].join('  ·  '),
              textAlign: TextAlign.center,
              style: TextStyle(color: p.confident ? Palette.ok : Palette.warning, fontWeight: FontWeight.w600),
            ),
            if (p.notes.isNotEmpty) ...[
              const SizedBox(height: 6),
              Text(p.notes, textAlign: TextAlign.center, style: const TextStyle(color: Palette.muted, fontSize: 13)),
            ],
          ] else if (reading)
            const Row(children: [
              SizedBox(width: 18, height: 18, child: CircularProgressIndicator(strokeWidth: 2)),
              SizedBox(width: 12),
              Text('Reading the plate with OpenAI…'),
            ])
          else
            Text(
              i.plateError ?? (p != null ? 'No readable plate. ${p.notes}' : 'Not read yet.'),
              style: TextStyle(color: i.plateStatus == PlateStatus.failed ? Palette.warning : Palette.muted),
            ),
          const SizedBox(height: 12),
          OutlinedButton.icon(
            onPressed: reading || _busy ? null : () => _run(() => app.closePass.readPlate(i)),
            icon: const Icon(Icons.auto_awesome, size: 18),
            label: Text(p == null ? 'Read plate' : 'Read again'),
          ),
        ]),
      ),
    );
  }

  Widget _photoStrip(Incident i) {
    if (_photos.isEmpty) {
      return const Card(child: Padding(padding: EdgeInsets.all(18), child: Text('No photos were received.', style: TextStyle(color: Palette.muted))));
    }
    return SizedBox(
      height: 132,
      child: ListView.separated(
        scrollDirection: Axis.horizontal,
        itemCount: _photos.length,
        separatorBuilder: (_, _) => const SizedBox(width: 10),
        itemBuilder: (context, n) {
          final best = n == i.bestPhotoIndex && i.plate?.found == true;
          return GestureDetector(
            onTap: () => _showPhoto(context, n, i),
            child: Container(
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(16),
                border: Border.all(color: best ? Palette.accent : Palette.outline, width: best ? 2.5 : 1),
              ),
              clipBehavior: Clip.antiAlias,
              child: Stack(children: [
                Image.memory(_photos[n], height: 132, fit: BoxFit.cover, gaplessPlayback: true),
                Positioned(
                  left: 6,
                  bottom: 6,
                  child: Container(
                    padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                    decoration: BoxDecoration(color: Colors.black54, borderRadius: BorderRadius.circular(6)),
                    child: Text(
                      '${n + 1}${n < i.photoOffsetsMs.length ? '  +${(i.photoOffsetsMs[n] / 1000).toStringAsFixed(1)} s' : ''}${best ? '  · plate' : ''}',
                      style: const TextStyle(fontSize: 11, color: Colors.white),
                    ),
                  ),
                ),
              ]),
            ),
          );
        },
      ),
    );
  }

  void _showPhoto(BuildContext context, int n, Incident i) {
    showDialog<void>(
      context: context,
      builder: (c) => Dialog(
        backgroundColor: Colors.black,
        insetPadding: const EdgeInsets.all(12),
        child: Column(mainAxisSize: MainAxisSize.min, children: [
          InteractiveViewer(maxScale: 5, child: Image.memory(_photos[n], fit: BoxFit.contain)),
          Padding(
            padding: const EdgeInsets.all(10),
            child: Text('Photo ${n + 1} of ${_photos.length}  ·  pinch to zoom', style: const TextStyle(color: Palette.muted)),
          ),
        ]),
      ),
    );
  }

  Widget _uploadCard(AppServices app, Incident i) {
    final configured = app.settings.uploadUri != null;
    final text = switch (i.uploadStatus) {
      UploadStatus.uploaded => 'Uploaded${i.uploadedAt == null ? '' : ' ${formatWhen(i.uploadedAt!)}'}.',
      UploadStatus.uploading => 'Uploading…',
      UploadStatus.failed => i.uploadError ?? 'Upload failed.',
      UploadStatus.notConfigured => 'Add the website upload address in Settings to upload.',
      UploadStatus.waiting => 'Waiting to upload.',
      UploadStatus.notNeeded => i.plate?.confident == true
          ? 'Ready to upload.'
          : 'Uploads happen automatically when a plate is read confidently. You can also upload by hand.',
    };
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(18),
        child: Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: [
          Row(children: [
            UploadIcon(status: i.uploadStatus),
            if (i.uploadStatus != UploadStatus.notNeeded) const SizedBox(width: 10),
            Expanded(child: Text(text, style: const TextStyle(color: Palette.muted))),
          ]),
          if (i.uploadedUrl != null) ...[
            const SizedBox(height: 8),
            SelectableText(i.uploadedUrl!, style: const TextStyle(color: Palette.accent)),
            TextButton.icon(
              onPressed: () => Clipboard.setData(ClipboardData(text: i.uploadedUrl!)),
              icon: const Icon(Icons.copy, size: 16),
              label: const Text('Copy link'),
            ),
          ],
          const SizedBox(height: 12),
          FilledButton.icon(
            onPressed: !configured || _busy || i.uploadStatus == UploadStatus.uploading ? null : () => _run(() => app.closePass.upload(i)),
            icon: const Icon(Icons.cloud_upload_outlined),
            label: Text(i.uploadStatus == UploadStatus.uploaded ? 'Upload again' : 'Upload to website'),
          ),
        ]),
      ),
    );
  }
}

/// A number plate drawn like the real thing (Irish style: white with a blue band).
class _PlateBadge extends StatelessWidget {
  final String plate;
  final bool irish;
  const _PlateBadge({required this.plate, required this.irish});

  @override
  Widget build(BuildContext context) => Container(
        decoration: BoxDecoration(
          color: irish ? const Color(0xFFF7F7F2) : const Color(0xFFF6C32A),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: Colors.black, width: 2),
        ),
        clipBehavior: Clip.antiAlias,
        child: IntrinsicHeight(
          child: Row(mainAxisSize: MainAxisSize.min, children: [
            if (irish)
              Container(
                width: 26,
                color: const Color(0xFF1546A0),
                alignment: Alignment.bottomCenter,
                padding: const EdgeInsets.only(bottom: 4),
                child: const Text('IRL', style: TextStyle(color: Colors.white, fontSize: 9, fontWeight: FontWeight.w800)),
              ),
            Flexible(
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                child: FittedBox(
                  fit: BoxFit.scaleDown,
                  child: Text(plate,
                      style: const TextStyle(color: Colors.black, fontSize: 30, fontWeight: FontWeight.w900, letterSpacing: 2)),
                ),
              ),
            ),
          ]),
        ),
      );
}
